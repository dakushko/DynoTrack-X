#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <math.h>

// DynoTrack X - day-1 skeleton (sync stack)
// - ESP32-S3 first tries to join OBDII WiFi STA: "WIFI_OBDII."
// - ESP32-S3 also exposes WiFi AP: "DynoTrack-X" (open)
// - HTTP server on :80 serves a minimal dashboard at "/"
// - WebSocket server on :81 pushes dummy live JSON at 10 Hz

static const char* kObdSsid = "WIFI_OBDII.";
static const char* kApSsid = "DynoTrack-X";
static const bool kUseDummyObd = true; // keep true until physical OBDII arrives
static const uint16_t kHttpPort = 80;
static const uint16_t kWsPort = 81;

static WebServer httpServer(kHttpPort);
static WebSocketsServer wsServer(kWsPort);

static Preferences prefs;

static float g_weightKg = 0.0f;
static String g_tireSize = "";
static float g_humidityPct = NAN;
static float g_pressureHpa = NAN;
static bool g_unitsMetric = true;
static float g_finalDriveRatio = 4.10f;
static float g_gearRatio = 3.50f;
static float g_gearboxLossPct = 8.0f;
static String g_driveType = "fwd";
static float g_dragCd = 0.31f;
static float g_frontalAreaM2 = 2.20f;
static float g_rollResCoeff = 0.015f;
static float g_roadGradePct = 0.0f;
static float g_wheelRadiusM = 0.315f;
static String g_corrStandard = "din";

static bool g_setupOk = false;
static String g_missingFieldsJson = "[]";

struct Kalman1D {
  float x;
  float p;
  float q;
  float r;
  bool initialized;
};

static Kalman1D g_kfRpm = {0, 1, 0.6f, 20.0f, false};
static Kalman1D g_kfSpeed = {0, 1, 0.2f, 2.0f, false};
static Kalman1D g_kfThrottle = {0, 1, 0.5f, 4.0f, false};
static Kalman1D g_kfFuelRate = {0, 1, 0.2f, 1.5f, false};

static float kalmanUpdate(Kalman1D& kf, float z) {
  if (!kf.initialized) {
    kf.x = z;
    kf.p = 1.0f;
    kf.initialized = true;
    return kf.x;
  }
  kf.p += kf.q;
  const float k = kf.p / (kf.p + kf.r);
  kf.x = kf.x + k * (z - kf.x);
  kf.p = (1.0f - k) * kf.p;
  return kf.x;
}

static void recomputeSetupState() {
  bool ok = true;
  String missing = "[";
  bool first = true;

  auto addMissing = [&](const char* key) {
    if (!first) missing += ",";
    missing += "\"";
    missing += key;
    missing += "\"";
    first = false;
    ok = false;
  };

  if (g_weightKg <= 0.1f) addMissing("weightKg");
  if (g_tireSize.length() < 3) addMissing("tireSize");

  missing += "]";
  g_setupOk = ok;
  g_missingFieldsJson = missing;
}

static void loadSettings() {
  prefs.begin("dyntx", true);
  g_weightKg = prefs.getFloat("weightKg", 0.0f);
  g_tireSize = prefs.getString("tireSize", "");

  float hum = prefs.getFloat("humidityPct", NAN);
  float pres = prefs.getFloat("pressureHpa", NAN);
  g_humidityPct = isnan(hum) ? NAN : hum;
  g_pressureHpa = isnan(pres) ? NAN : pres;

  g_unitsMetric = prefs.getBool("unitsMetric", true);
  g_finalDriveRatio = prefs.getFloat("finalDrive", 4.10f);
  g_gearRatio = prefs.getFloat("gearRatio", 3.50f);
  g_gearboxLossPct = prefs.getFloat("gbLossPct", 8.0f);
  g_driveType = prefs.getString("driveType", "fwd");
  g_dragCd = prefs.getFloat("dragCd", 0.31f);
  g_frontalAreaM2 = prefs.getFloat("frAreaM2", 2.20f);
  g_rollResCoeff = prefs.getFloat("rollRes", 0.015f);
  g_roadGradePct = prefs.getFloat("gradePct", 0.0f);
  g_wheelRadiusM = prefs.getFloat("wheelRadM", 0.315f);
  g_corrStandard = prefs.getString("corrStd", "din");
  prefs.end();

  recomputeSetupState();
}

static bool parseMandatoryFields() {
  if (g_weightKg <= 0.1f) return false;
  if (g_tireSize.length() < 3) return false;
  return true;
}

static String jsonApiSettings() {
  char buf[560];
  snprintf(buf, sizeof(buf),
           "{\"setup_ok\":%s,\"weightKg\":%.2f,\"tireSize\":\"%s\",\"humidityPct\":%.1f,\"pressureHpa\":%.1f,\"unitsMetric\":%s,\"finalDriveRatio\":%.3f,\"gearRatio\":%.3f,\"drivetrainLossPct\":%.1f,\"dragCd\":%.3f,\"frontalAreaM2\":%.3f,\"rollResCoeff\":%.4f,\"roadGradePct\":%.2f,\"wheelRadiusM\":%.4f,\"driveType\":\"%s\",\"corrStandard\":\"%s\",\"missing_fields\":%s}",
           g_setupOk ? "true" : "false",
           (double)g_weightKg,
           g_tireSize.c_str(),
           isnan(g_humidityPct) ? 0.0 : (double)g_humidityPct,
           isnan(g_pressureHpa) ? 0.0 : (double)g_pressureHpa,
           g_unitsMetric ? "true" : "false",
           (double)g_finalDriveRatio,
           (double)g_gearRatio,
           (double)g_gearboxLossPct,
           (double)g_dragCd,
           (double)g_frontalAreaM2,
           (double)g_rollResCoeff,
           (double)g_roadGradePct,
           (double)g_wheelRadiusM,
           g_driveType.c_str(),
           g_corrStandard.c_str(),
           g_missingFieldsJson.c_str());
  return String(buf);
}

static const char kHomeHtml[] PROGMEM = R"HTML(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1"/>
    <title>DynoTrack X</title>
    <style>
      :root { color-scheme: dark; }
      body {
        margin: 0;
        font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
        background: #000000;
        color: #ffffff;
      }
      header {
        padding: 16px 18px;
        border-bottom: 1px solid rgba(255,255,255,0.16);
        display: grid;
        grid-template-columns: 1fr auto 1fr;
        align-items: start;
        gap: 12px;
      }
      .headerLeft {
        display: flex;
        justify-content: flex-start;
      }
      .headerCenter {
        text-align: center;
      }
      .headerRight {
        display: flex;
        justify-content: flex-end;
        align-items: center;
      }
      header .title { font-weight: 900; letter-spacing: 0.4px; color: #ffffff; }
      header .sub { font-size: 12px; opacity: 0.95; color: #f3f6ff; }
      .wsBadge {
        display: inline-block;
        font-size: 12px;
        font-weight: 700;
        opacity: 1;
        color: #ffffff;
        border: 1px solid rgba(255,255,255,0.45);
        background: rgba(255,255,255,0.12);
        padding: 4px 9px;
        border-radius: 999px;
      }
      .settingsLink {
        font-size: 12px;
        opacity: 1;
        font-weight: 800;
        text-decoration: none;
        color: #ffffff;
        border: 1px solid rgba(255,255,255,0.45);
        background: rgba(255,255,255,0.12);
        padding: 6px 10px;
        border-radius: 999px;
      }
      .wrap { padding: 16px 18px 24px; }
      .grid {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 14px;
      }
      @media (max-width: 420px) { .grid { grid-template-columns: 1fr; } }
      .card {
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 14px 14px 16px;
        background: rgba(255,255,255,0.06);
      }
      .label { font-size: 12px; opacity: 1; margin-bottom: 6px; color: #f2f6ff; }
      .value {
        font-size: 34px;
        font-weight: 900;
        letter-spacing: 0.2px;
        color: #ffffff;
      }
      .unit { font-size: 12px; opacity: 1; font-weight: 700; margin-left: 6px; color: #f5f8ff;}
      .status {
        margin-top: 14px;
        font-size: 12px;
        opacity: 0.8;
        display: flex;
        gap: 10px;
        flex-wrap: wrap;
      }
      .pill {
        border: 1px solid rgba(255,255,255,0.28);
        background: rgba(255,255,255,0.10);
        padding: 6px 10px;
        border-radius: 999px;
        color: #ffffff;
      }
      .health-normal {
        border-color: rgba(120,255,170,0.5);
        color: #b9f6ca;
      }
      .health-warn {
        border-color: rgba(255,210,120,0.6);
        color: #ffd98e;
      }
      .health-hot {
        border-color: rgba(255,120,120,0.65);
        color: #ffb3b3;
      }
      .banner {
        margin: 14px 0 0;
        padding: 12px 14px;
        border: 1px solid rgba(255,80,80,0.35);
        background: rgba(255,80,80,0.08);
        border-radius: 14px;
        display: none;
      }
      .banner strong { color: #ffb1b1; }
      .banner a {
        color: #b9f6ca;
        text-decoration: none;
        font-weight: 700;
        margin-left: 10px;
      }
      .controls { margin-top: 14px; display: flex; gap: 10px; flex-wrap: wrap; }
      .btn {
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(255,255,255,0.12);
        color: #fff;
        border-radius: 10px;
        padding: 10px 14px;
        font-weight: 800;
      }
      .chartCard, .runsCard, .reportCard {
        margin-top: 14px;
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 12px;
        background: rgba(255,255,255,0.06);
      }
      #dynoCanvas { width: 100%; height: 260px; background: #050505; border-radius: 10px; }
      .reportTable { width: 100%; border-collapse: collapse; font-size: 12px; margin-top: 8px; }
      .reportTable td { border-bottom: 1px solid rgba(255,255,255,0.12); padding: 6px 4px; }
      .runsList { margin-top: 8px; font-size: 12px; }
      .runItem { padding: 6px 0; border-bottom: 1px solid rgba(255,255,255,0.12); }
      .unitPill {
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(255,255,255,0.12);
        border-radius: 999px;
        padding: 8px 10px;
        font-size: 12px;
        font-weight: 800;
      }
      .printHeader { display: none; }
      @media print {
        body { background: #fff; color: #000; }
        header, .controls, .runsCard, .status, .settingsLink, .banner { display: none !important; }
        .wrap { padding: 0; }
        .chartCard, .reportCard {
          border: 1px solid #000;
          background: #fff;
          color: #000;
          page-break-inside: avoid;
        }
        .label, .value, .unit, .pill, .msg { color: #000 !important; }
        #dynoCanvas { background: #fff; border: 1px solid #000; }
        .printHeader {
          display: block;
          margin-bottom: 10px;
          border-bottom: 1px solid #000;
          padding-bottom: 8px;
        }
      }
    </style>
  </head>
  <body>
    <header>
      <div class="headerLeft">
        <div id="wsState" class="wsBadge">WS: connecting...</div>
      </div>
      <div class="headerCenter">
        <div class="title">DynoTrack X</div>
        <div class="sub">WebSocket live skeleton (10 Hz dummy data)</div>
      </div>
      <div class="headerRight">
        <a class="settingsLink" href="/settings">Settings</a>
      </div>
    </header>

    <div class="wrap">
      <div class="printHeader">
        <h2>DynoTrack X - Dyno Report</h2>
        <div id="printMeta">Generated: -</div>
      </div>
      <div class="banner" id="setupBanner">
        <strong id="setupTitle">Setup required</strong>
        <span id="setupMsg"></span>
        <a href="/settings">Settings</a>
      </div>

      <div class="grid">
        <div class="card">
          <div class="label">Speed</div>
          <div class="value"><span id="speed">0</span><span class="unit">km/h</span></div>
        </div>

        <div class="card">
          <div class="label">RPM (OBDII)</div>
          <div class="value"><span id="rpm">0</span><span class="unit">rpm</span></div>
        </div>

        <div class="card">
          <div class="label">HP</div>
          <div class="value"><span id="hp">0</span><span id="unitHp" class="unit">hp</span></div>
        </div>

        <div class="card">
          <div class="label">Torque</div>
          <div class="value"><span id="torque">0</span><span class="unit">Nm</span></div>
        </div>

        <div class="card">
          <div class="label">Power @ Wheels</div>
          <div class="value"><span id="hpWheel">0</span><span id="unitWheel" class="unit">hp</span></div>
        </div>

        <div class="card">
          <div class="label">Power @ Crank</div>
          <div class="value"><span id="hpCrank">0</span><span id="unitCrank" class="unit">hp</span></div>
        </div>

        <div class="card">
          <div class="label">Corrected Power</div>
          <div class="value"><span id="hpCorr">0</span><span id="unitCorr" class="unit">hp</span></div>
        </div>

        <div class="card">
          <div class="label">Throttle (OBDII)</div>
          <div class="value"><span id="throttle">0</span><span class="unit">%</span></div>
        </div>

        <div class="card">
          <div class="label">Fuel Rate</div>
          <div class="value"><span id="fuelRate">0</span><span class="unit">L/h</span></div>
        </div>

        <div class="card" id="iatCard">
          <div class="label">Air Intake Temp (OBDII)</div>
          <div class="value"><span id="iat">0</span><span class="unit">C</span></div>
        </div>

        <div class="card" id="oilCard">
          <div class="label">Engine Oil Temp (OBDII)</div>
          <div class="value"><span id="oilTemp">0</span><span class="unit">C</span></div>
        </div>
      </div>

      <div class="status">
        <div class="pill">Dummy push: 10 Hz</div>
        <div class="pill" id="tInfo">t_ms: 0</div>
        <div class="pill" id="lossInfo">Loss total: 0.0 hp</div>
        <div class="pill" id="corrInfo">Corr: DIN x1.000</div>
        <div class="pill" id="anomalyInfo">Anomaly: none</div>
        <div class="pill" id="iatHealth">IAT: normal</div>
        <div class="pill" id="oilHealth">Oil: normal</div>
      </div>

      <div class="controls">
        <button id="btnStart" class="btn">START</button>
        <button id="btnSaveRun" class="btn">SAVE RUN</button>
        <button id="btnExportCsv" class="btn">EXPORT CSV</button>
        <button id="btnPrintReport" class="btn">PRINT REPORT</button>
        <button id="btnTogglePowerUnit" class="unitPill">Power Unit: HP</button>
      </div>

      <div class="chartCard">
        <div class="label">Dyno Graph (RPM vs HP/Nm)</div>
        <canvas id="dynoCanvas" width="900" height="260"></canvas>
      </div>

      <div class="reportCard">
        <div class="label">Dyno Report Summary</div>
        <table class="reportTable">
          <tr><td>Peak Power</td><td id="peakPowerInfo">-</td></tr>
          <tr><td>Peak Torque</td><td id="peakTorqueInfo">-</td></tr>
          <tr><td>WHP vs Engine HP</td><td id="wheelVsEngineInfo">-</td></tr>
          <tr><td>Correction</td><td id="corrSummaryInfo">-</td></tr>
          <tr><td>Ambient (T/P/H)</td><td id="ambientInfo">-</td></tr>
        </table>
      </div>

      <div class="runsCard">
        <div class="label">Saved Runs</div>
        <div id="runsList" class="runsList"></div>
      </div>
    </div>

    <script>
      const wsUrl = 'ws://' + location.hostname + ':81/';
      const wsState = document.getElementById('wsState');
      const elSpeed = document.getElementById('speed');
      const elRpm = document.getElementById('rpm');
      const elHp = document.getElementById('hp');
      const elTorque = document.getElementById('torque');
      const elHpWheel = document.getElementById('hpWheel');
      const elHpCrank = document.getElementById('hpCrank');
      const elHpCorr = document.getElementById('hpCorr');
      const unitHp = document.getElementById('unitHp');
      const unitWheel = document.getElementById('unitWheel');
      const unitCrank = document.getElementById('unitCrank');
      const unitCorr = document.getElementById('unitCorr');
      const elThrottle = document.getElementById('throttle');
      const elFuelRate = document.getElementById('fuelRate');
      const elIat = document.getElementById('iat');
      const elOilTemp = document.getElementById('oilTemp');
      const elLossInfo = document.getElementById('lossInfo');
      const elCorrInfo = document.getElementById('corrInfo');
      const elAnomalyInfo = document.getElementById('anomalyInfo');
      const elIatHealth = document.getElementById('iatHealth');
      const elOilHealth = document.getElementById('oilHealth');
      const iatCard = document.getElementById('iatCard');
      const oilCard = document.getElementById('oilCard');
      const elTInfo = document.getElementById('tInfo');
      const btnStart = document.getElementById('btnStart');
      const btnSaveRun = document.getElementById('btnSaveRun');
      const btnExportCsv = document.getElementById('btnExportCsv');
      const btnPrintReport = document.getElementById('btnPrintReport');
      const btnTogglePowerUnit = document.getElementById('btnTogglePowerUnit');
      const printMeta = document.getElementById('printMeta');
      const runsList = document.getElementById('runsList');
      const peakPowerInfo = document.getElementById('peakPowerInfo');
      const peakTorqueInfo = document.getElementById('peakTorqueInfo');
      const wheelVsEngineInfo = document.getElementById('wheelVsEngineInfo');
      const corrSummaryInfo = document.getElementById('corrSummaryInfo');
      const ambientInfo = document.getElementById('ambientInfo');
      const canvas = document.getElementById('dynoCanvas');
      const ctx = canvas.getContext('2d');

      const setupBanner = document.getElementById('setupBanner');
      const setupMsg = document.getElementById('setupMsg');
      let runArmed = false;
      let runActive = false;
      let currentRun = [];
      let savedRuns = [];
      const maxRunPoints = 2200;
      let powerUnit = 'hp';
      let lastLiveMsg = null;

      function powerConvert(vHp) {
        return powerUnit === 'kw' ? (vHp * 0.7457) : vHp;
      }

      function powerUnitLabel() {
        return powerUnit === 'kw' ? 'kW' : 'hp';
      }

      function refreshPowerUnitLabels() {
        const u = powerUnitLabel();
        unitHp.textContent = u;
        unitWheel.textContent = u;
        unitCrank.textContent = u;
        unitCorr.textContent = u;
      }

      function updateSetup(msg) {
        if (msg.setup_ok) {
          setupBanner.style.display = 'none';
          return;
        }
        const missing = Array.isArray(msg.missing_fields) ? msg.missing_fields : [];
        const labels = { weightKg: 'vehicle weight', tireSize: 'tire size' };
        const pretty = missing.map(k => labels[k] || k).join(', ');
        setupMsg.textContent = pretty ? (': ' + pretty) : '';
        setupBanner.style.display = 'block';
      }

      function setHealthVisual(level, pillEl, cardEl, labelPrefix) {
        pillEl.classList.remove('health-normal', 'health-warn', 'health-hot');
        cardEl.classList.remove('health-normal', 'health-warn', 'health-hot');
        const cls = level === 'hot' ? 'health-hot' : (level === 'warn' ? 'health-warn' : 'health-normal');
        pillEl.classList.add(cls);
        cardEl.classList.add(cls);
        pillEl.textContent = labelPrefix + ': ' + level;
      }

      function classifyIat(tempC) {
        if (tempC >= 55) return 'hot';
        if (tempC >= 40) return 'warn';
        return 'normal';
      }

      function classifyOil(tempC) {
        if (tempC >= 125) return 'hot';
        if (tempC >= 110) return 'warn';
        return 'normal';
      }

      function beep(freq = 920, ms = 120) {
        try {
          const ac = new (window.AudioContext || window.webkitAudioContext)();
          const o = ac.createOscillator();
          const g = ac.createGain();
          o.frequency.value = freq;
          o.connect(g);
          g.connect(ac.destination);
          g.gain.value = 0.05;
          o.start();
          setTimeout(() => { o.stop(); ac.close(); }, ms);
        } catch (e) {}
      }

      function say(text) {
        try {
          if (!('speechSynthesis' in window)) return;
          const u = new SpeechSynthesisUtterance(text);
          u.lang = 'en-US';
          u.rate = 1.05;
          u.pitch = 1.0;
          window.speechSynthesis.cancel();
          window.speechSynthesis.speak(u);
        } catch (e) {}
      }

      function renderRuns() {
        if (!savedRuns.length) {
          runsList.textContent = 'No saved runs yet.';
          return;
        }
        runsList.innerHTML = savedRuns.map((r, i) => {
          const peak = r.points.reduce((m, p) => Math.max(m, p.hp_corrected || 0), 0);
          return '<div class="runItem">#' + (i + 1) + ' | ' + r.startedAt
            + ' | peak corrected: ' + peak.toFixed(1) + ' hp'
            + ' | points: ' + r.points.length + '</div>';
        }).join('');
      }

      function persistRuns() {
        localStorage.setItem('dynotrack_runs_v1', JSON.stringify(savedRuns.slice(-25)));
      }

      function loadRuns() {
        try {
          const raw = localStorage.getItem('dynotrack_runs_v1');
          savedRuns = raw ? JSON.parse(raw) : [];
        } catch (e) {
          savedRuns = [];
        }
        renderRuns();
      }

      function drawCurve(points) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = '#060606';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        if (!points.length) return;
        const left = 52, right = canvas.width - 48, top = 14, bottom = canvas.height - 24;
        const minR = 1000, maxR = 8000;
        let maxHp = 1, maxTq = 1, maxLoss = 1;
        points.forEach(p => {
          maxHp = Math.max(maxHp, powerConvert(p.hp_corrected || 0));
          maxTq = Math.max(maxTq, p.torque_nm || 0);
          maxLoss = Math.max(maxLoss, powerConvert(p.hp_loss_total || 0));
        });
        maxHp = Math.ceil(maxHp / 10) * 10;
        maxTq = Math.ceil(maxTq / 10) * 10;
        maxLoss = Math.ceil(maxLoss / 10) * 10;
        const maxPowerAxis = Math.max(maxHp, maxLoss);
        const mapX = r => ((Math.min(maxR, Math.max(minR, r)) - minR) / (maxR - minR)) * (right - left) + left;
        const mapYHp = h => bottom - (h / maxPowerAxis) * (bottom - top);
        const mapYLoss = h => bottom - (h / maxPowerAxis) * (bottom - top);
        const mapYTq = t => bottom - (t / maxTq) * (bottom - top);

        // Grid + graduated axis ticks
        ctx.strokeStyle = 'rgba(255,255,255,0.12)';
        ctx.fillStyle = 'rgba(220,232,255,0.90)';
        ctx.font = '10px Arial';
        for (let rpmTick = 1000; rpmTick <= 8000; rpmTick += 1000) {
          const x = mapX(rpmTick);
          ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, bottom); ctx.stroke();
          ctx.fillText(String(rpmTick), x - 12, bottom + 14);
        }
        const yTicks = 6;
        for (let i = 0; i <= yTicks; i++) {
          const y = top + ((bottom - top) * i) / yTicks;
          ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
          const leftVal = ((maxPowerAxis * (yTicks - i)) / yTicks);
          const rightVal = ((maxTq * (yTicks - i)) / yTicks);
          ctx.fillText(leftVal.toFixed(0), 18, y + 3);
          ctx.fillText(rightVal.toFixed(0), right + 6, y + 3);
        }

        ctx.strokeStyle = 'rgba(255,255,255,0.25)';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(left, top); ctx.lineTo(left, bottom); ctx.lineTo(right, bottom); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(right, top); ctx.lineTo(right, bottom); ctx.stroke();
        ctx.fillStyle = '#cfe2ff'; ctx.font = '11px Arial';
        ctx.fillText(powerUnit === 'kw' ? 'kW / Loss' : 'HP / Loss', 8, 20);
        ctx.fillText('RPM', (canvas.width / 2) - 14, canvas.height - 6);
        ctx.fillText('Nm', canvas.width - 28, 20);
        ctx.fillText(powerUnit === 'kw' ? 'kW' : 'HP', left + 4, top + 12);
        ctx.fillStyle = '#7db2ff';
        ctx.fillText('Torque', left + 38, top + 12);
        ctx.fillStyle = '#ff8aa0';
        ctx.fillText('Loss', left + 90, top + 12);

        ctx.strokeStyle = '#00ffa0';
        ctx.beginPath();
        points.forEach((p, i) => { const x = mapX(p.rpm || 0), y = mapYHp(powerConvert(p.hp_corrected || 0)); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
        ctx.stroke();
        ctx.strokeStyle = '#7db2ff';
        ctx.beginPath();
        points.forEach((p, i) => { const x = mapX(p.rpm || 0), y = mapYTq(p.torque_nm || 0); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
        ctx.stroke();
        ctx.strokeStyle = '#ff8aa0';
        ctx.beginPath();
        points.forEach((p, i) => { const x = mapX(p.rpm || 0), y = mapYLoss(powerConvert(p.hp_loss_total || 0)); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
        ctx.stroke();
      }

      function updateReportFromPoints(points, msg) {
        if (!points.length) return;
        let peakHp = { v: 0, rpm: 0 };
        let peakTq = { v: 0, rpm: 0 };
        points.forEach(p => {
          if (powerConvert(p.hp_corrected || 0) > peakHp.v) peakHp = { v: powerConvert(p.hp_corrected || 0), rpm: p.rpm || 0 };
          if ((p.torque_nm || 0) > peakTq.v) peakTq = { v: p.torque_nm || 0, rpm: p.rpm || 0 };
        });
        const last = points[points.length - 1];
        peakPowerInfo.textContent = peakHp.v.toFixed(1) + ' ' + powerUnitLabel() + ' @ ' + peakHp.rpm.toFixed(0) + ' RPM';
        peakTorqueInfo.textContent = peakTq.v.toFixed(1) + ' Nm @ ' + peakTq.rpm.toFixed(0) + ' RPM';
        wheelVsEngineInfo.textContent = powerConvert(last.hp_wheel || 0).toFixed(1) + ' ' + powerUnitLabel()
          + ' wheel / ' + powerConvert(last.hp_crank || 0).toFixed(1) + ' ' + powerUnitLabel() + ' engine';
        corrSummaryInfo.textContent = String(msg.corr_std || 'DIN').toUpperCase() + ' | K=' + Number(msg.corr_factor_k || 1).toFixed(3);
        ambientInfo.textContent = Number(msg.air_intake_c || 0).toFixed(1) + ' C, '
          + Number(msg.pressure_hpa || 1013.2).toFixed(1) + ' hPa, '
          + Number(msg.humidity_pct || 0).toFixed(1) + ' %';
      }

      function saveCurrentRun() {
        if (!currentRun.length) return;
        savedRuns.push({ startedAt: new Date().toISOString(), points: currentRun.slice() });
        persistRuns();
        renderRuns();
        const peak = currentRun.reduce((m, p) => Math.max(m, p.hp_corrected || 0), 0);
        say('Run saved. Peak power ' + peak.toFixed(0) + ' horsepower.');
        beep(1040, 120);
      }

      function exportRunsCsv() {
        const rows = ['run,idx,t_ms,rpm,speed_kmh,hp_wheel,hp_crank,hp_corrected,torque_nm,throttle_pct,fuel_rate_lph,air_intake_c,engine_oil_c,anomaly'];
        savedRuns.forEach((r, ri) => r.points.forEach((p, i) => rows.push([
          ri + 1, i, p.t_ms, p.rpm, p.speed_kmh, p.hp_wheel, p.hp_crank, p.hp_corrected, p.torque_nm, p.throttle_pct, p.fuel_rate_lph, p.air_intake_c, p.engine_oil_c, p.anomaly
        ].join(','))));
        const blob = new Blob([rows.join('\n')], { type: 'text/csv' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'dynotrack_runs.csv';
        a.click();
      }

      let ws = null;
      let reconnectTimer = null;
      let reconnectDelayMs = 600;

      function scheduleReconnect() {
        if (reconnectTimer) return;
        reconnectTimer = setTimeout(() => {
          reconnectTimer = null;
          connectWs();
        }, reconnectDelayMs);
        reconnectDelayMs = Math.min(4000, reconnectDelayMs + 400);
      }

      function connectWs() {
        wsState.textContent = 'WS: connecting...';
        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
          reconnectDelayMs = 600;
          wsState.textContent = 'WS: connected';
        };
        ws.onclose = () => {
          wsState.textContent = 'WS: reconnecting...';
          scheduleReconnect();
        };
        ws.onerror = () => {
          wsState.textContent = 'WS: error';
        };

        ws.onmessage = (event) => {
          const msg = JSON.parse(event.data);
          if (!msg || msg.type !== 'live') return;
          updateSetup(msg);
          elSpeed.textContent = Number(msg.speed_kmh).toFixed(0);
          elRpm.textContent = Number(msg.rpm).toFixed(0);
          elHp.textContent = powerConvert(Number(msg.hp)).toFixed(0);
          elTorque.textContent = Number(msg.torque_nm).toFixed(0);
          elHpWheel.textContent = powerConvert(Number(msg.hp_wheel)).toFixed(0);
          elHpCrank.textContent = powerConvert(Number(msg.hp_crank)).toFixed(0);
          elHpCorr.textContent = powerConvert(Number(msg.hp_corrected)).toFixed(0);
          elThrottle.textContent = Number(msg.throttle_pct).toFixed(0);
          elFuelRate.textContent = Number(msg.fuel_rate_lph).toFixed(1);
          const iatC = Number(msg.air_intake_c);
          const oilC = Number(msg.engine_oil_c);
          elIat.textContent = iatC.toFixed(0);
          elOilTemp.textContent = oilC.toFixed(0);
          elLossInfo.textContent = 'Loss total: ' + powerConvert(Number(msg.hp_loss_total)).toFixed(1)
            + ' ' + powerUnitLabel() + ' (GB ' + Number(msg.loss_gearbox_pct).toFixed(1) + '%, '
            + String(msg.drive_type || '').toUpperCase() + ')';
          elCorrInfo.textContent = 'Corr: ' + String(msg.corr_std || 'DIN').toUpperCase()
            + ' x' + Number(msg.corr_factor_k).toFixed(3);
          elAnomalyInfo.textContent = 'Anomaly: ' + String(msg.anomaly || 'none');
          setHealthVisual(classifyIat(iatC), elIatHealth, iatCard, 'IAT');
          setHealthVisual(classifyOil(oilC), elOilHealth, oilCard, 'Oil');
          elTInfo.textContent = 't_ms: ' + msg.t_ms;

          lastLiveMsg = msg;
          const sample = {
            t_ms: Number(msg.t_ms), rpm: Number(msg.rpm), speed_kmh: Number(msg.speed_kmh),
            hp_wheel: Number(msg.hp_wheel), hp_crank: Number(msg.hp_crank), hp_corrected: Number(msg.hp_corrected),
            hp_loss_total: Number(msg.hp_loss_total),
            torque_nm: Number(msg.torque_nm), throttle_pct: Number(msg.throttle_pct), fuel_rate_lph: Number(msg.fuel_rate_lph),
            air_intake_c: Number(msg.air_intake_c), engine_oil_c: Number(msg.engine_oil_c), anomaly: String(msg.anomaly || '')
          };
          const startTrigger = runArmed && !runActive && sample.throttle_pct > 70 && sample.rpm > 1800;
          const stopTrigger = runActive && (sample.throttle_pct < 25 || sample.rpm < 1500);
          if (startTrigger) {
            runActive = true;
            currentRun = [];
            say('Run started.');
            beep(1240, 130);
          }
          if (runActive) {
            currentRun.push(sample);
            if (currentRun.length > maxRunPoints) currentRun.shift();
            drawCurve(currentRun);
            updateReportFromPoints(currentRun, msg);
          }
          if (stopTrigger && currentRun.length > 10) {
            runActive = false;
            saveCurrentRun();
          }
        };
      }

      btnStart.addEventListener('click', () => {
        runArmed = !runArmed;
        if (!runArmed) runActive = false;
        btnStart.textContent = runArmed ? 'STARTED' : 'START';
        say(runArmed ? 'Start mode enabled.' : 'Start mode disabled.');
        beep(runArmed ? 900 : 620, 100);
      });
      btnSaveRun.addEventListener('click', saveCurrentRun);
      btnExportCsv.addEventListener('click', exportRunsCsv);
      btnPrintReport.addEventListener('click', () => {
        const dt = new Date().toLocaleString();
        printMeta.textContent = 'Generated: ' + dt;
        setTimeout(() => window.print(), 80);
      });
      btnTogglePowerUnit.addEventListener('click', () => {
        powerUnit = (powerUnit === 'hp') ? 'kw' : 'hp';
        btnTogglePowerUnit.textContent = 'Power Unit: ' + (powerUnit === 'hp' ? 'HP' : 'kW');
        refreshPowerUnitLabels();
        if (lastLiveMsg) {
          elHp.textContent = powerConvert(Number(lastLiveMsg.hp)).toFixed(0);
          elHpWheel.textContent = powerConvert(Number(lastLiveMsg.hp_wheel)).toFixed(0);
          elHpCrank.textContent = powerConvert(Number(lastLiveMsg.hp_crank)).toFixed(0);
          elHpCorr.textContent = powerConvert(Number(lastLiveMsg.hp_corrected)).toFixed(0);
          elLossInfo.textContent = 'Loss total: ' + powerConvert(Number(lastLiveMsg.hp_loss_total)).toFixed(1)
            + ' ' + powerUnitLabel() + ' (GB ' + Number(lastLiveMsg.loss_gearbox_pct).toFixed(1) + '%, '
            + String(lastLiveMsg.drive_type || '').toUpperCase() + ')';
        }
        if (currentRun.length) {
          drawCurve(currentRun);
          updateReportFromPoints(currentRun, lastLiveMsg || {});
        }
      });

      refreshPowerUnitLabels();
      loadRuns();
      connectWs();
    </script>
  </body>
</html>
)HTML";

static const char kSettingsHtml[] PROGMEM = R"HTML(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1"/>
    <title>DynoTrack X - Setup</title>
    <style>
      :root { color-scheme: dark; }
      body {
        margin: 0;
        font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
        background: #000000;
        color: #ffffff;
      }
      .wrap { padding: 18px; max-width: 520px; margin: 0 auto; }
      h1 { font-size: 18px; margin: 0 0 10px; }
      .card {
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 14px;
        background: rgba(255,255,255,0.06);
      }
      .row { margin: 12px 0; }
      label { display:block; font-size: 12px; opacity: 1; margin-bottom: 6px; color: #f2f6ff; }
      input, select {
        width: 100%;
        padding: 10px 12px;
        border-radius: 12px;
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(255,255,255,0.08);
        color: #ffffff;
        font-size: 14px;
      }
      select {
        background-color: #000000;
        color: #ffffff;
      }
      select option {
        background-color: #000000;
        color: #ffffff;
      }
      button {
        width: 100%;
        padding: 12px 14px;
        border-radius: 12px;
        border: none;
        background: #00ff88;
        color: #00110a;
        font-weight: 900;
        font-size: 14px;
      }
      .msg { font-size: 12px; opacity: 1; margin-top: 10px; color: #f0f4ff; }
      a { color: #ffffff; }
    </style>
  </head>
  <body>
    <div class="wrap">
      <h1>DynoTrack X Setup</h1>
      <div class="card">
        <div class="row">
          <label for="weightKg">Vehicle weight (kg) *</label>
          <input id="weightKg" type="number" min="1" step="1" placeholder="e.g. 1450"/>
        </div>

        <div class="row">
          <label for="tireSize">Tire size (e.g. 205/55R16) *</label>
          <input id="tireSize" type="text" placeholder="205/55R16"/>
        </div>

        <div class="row">
          <label for="humidityPct">Humidity % (optional)</label>
          <input id="humidityPct" type="number" step="0.1" placeholder="e.g. 55"/>
        </div>

        <div class="row">
          <label for="pressureHpa">Atmospheric pressure hPa (optional)</label>
          <input id="pressureHpa" type="number" step="0.1" placeholder="e.g. 1013"/>
        </div>

        <div class="row">
          <label for="units">Units</label>
          <select id="units">
            <option value="metric" selected>Metric (km/h, kg)</option>
            <option value="imperial">Imperial (mph, lb)</option>
          </select>
        </div>

        <div class="row" style="margin-top:18px;font-weight:800;">Advanced Dyno Inputs</div>

        <div class="row">
          <label for="finalDriveRatio">Final drive ratio</label>
          <input id="finalDriveRatio" type="number" step="0.001" placeholder="e.g. 4.100"/>
        </div>

        <div class="row">
          <label for="gearRatio">Current gear ratio</label>
          <input id="gearRatio" type="number" step="0.001" placeholder="e.g. 3.500"/>
        </div>

        <div class="row">
          <label for="drivetrainLossPct">Gearbox loss %</label>
          <input id="drivetrainLossPct" type="number" step="0.1" placeholder="e.g. 8"/>
        </div>

        <div class="row">
          <label for="driveType">Drive type</label>
          <select id="driveType">
            <option value="fwd" selected>FWD</option>
            <option value="rwd">RWD</option>
            <option value="awd">AWD</option>
          </select>
        </div>

        <div class="row">
          <label for="dragCd">Drag coefficient (Cd)</label>
          <input id="dragCd" type="number" step="0.001" placeholder="e.g. 0.310"/>
        </div>

        <div class="row">
          <label for="frontalAreaM2">Frontal area (m2)</label>
          <input id="frontalAreaM2" type="number" step="0.001" placeholder="e.g. 2.200"/>
        </div>

        <div class="row">
          <label for="rollResCoeff">Rolling resistance coeff</label>
          <input id="rollResCoeff" type="number" step="0.0001" placeholder="e.g. 0.0150"/>
        </div>

        <div class="row">
          <label for="roadGradePct">Road grade %</label>
          <input id="roadGradePct" type="number" step="0.01" placeholder="e.g. 0.00"/>
        </div>

        <div class="row">
          <label for="wheelRadiusM">Wheel radius (m)</label>
          <input id="wheelRadiusM" type="number" step="0.0001" placeholder="e.g. 0.3150"/>
        </div>

        <div class="row">
          <label for="corrStandard">Power correction standard</label>
          <select id="corrStandard">
            <option value="din" selected>DIN 70020</option>
            <option value="sae">SAE J1349</option>
          </select>
        </div>

        <button id="btnSave">Save</button>
        <div class="msg" id="status">Loading saved settings...</div>
        <div class="msg"><a href="/">Back to Home</a></div>
      </div>
    </div>

    <script>
      async function loadSettings() {
        const res = await fetch('/api/settings');
        const j = await res.json();
        const driveTypeEl = document.getElementById('driveType');
        driveTypeEl.innerHTML = ''
          + '<option value="fwd">FWD</option>'
          + '<option value="rwd">RWD</option>'
          + '<option value="awd">AWD</option>';
        document.getElementById('weightKg').value = j.weightKg ? j.weightKg : '';
        document.getElementById('tireSize').value = j.tireSize ? j.tireSize : '';
        document.getElementById('humidityPct').value = (j.humidityPct && j.humidityPct > 0) ? j.humidityPct : '';
        document.getElementById('pressureHpa').value = (j.pressureHpa && j.pressureHpa > 0) ? j.pressureHpa : '';
        document.getElementById('units').value = j.unitsMetric ? 'metric' : 'imperial';
        document.getElementById('finalDriveRatio').value = Number(j.finalDriveRatio || 4.1).toFixed(3);
        document.getElementById('gearRatio').value = Number(j.gearRatio || 3.5).toFixed(3);
        document.getElementById('drivetrainLossPct').value = Number(j.drivetrainLossPct || 8).toFixed(1);
        driveTypeEl.value = (j.driveType || 'fwd');
        document.getElementById('dragCd').value = Number(j.dragCd || 0.31).toFixed(3);
        document.getElementById('frontalAreaM2').value = Number(j.frontalAreaM2 || 2.2).toFixed(3);
        document.getElementById('rollResCoeff').value = Number(j.rollResCoeff || 0.015).toFixed(4);
        document.getElementById('roadGradePct').value = Number(j.roadGradePct || 0).toFixed(2);
        document.getElementById('wheelRadiusM').value = Number(j.wheelRadiusM || 0.315).toFixed(4);
        document.getElementById('corrStandard').value = (j.corrStandard || 'din');
        document.getElementById('status').textContent = j.setup_ok ? 'Setup OK' : 'Setup not complete yet';
      }

      async function saveSettings() {
        const weightKg = document.getElementById('weightKg').value;
        const tireSize = document.getElementById('tireSize').value;
        const humidityPct = document.getElementById('humidityPct').value;
        const pressureHpa = document.getElementById('pressureHpa').value;
        const units = document.getElementById('units').value;
        const finalDriveRatio = document.getElementById('finalDriveRatio').value;
        const gearRatio = document.getElementById('gearRatio').value;
        const drivetrainLossPct = document.getElementById('drivetrainLossPct').value;
        const driveType = document.getElementById('driveType').value;
        const dragCd = document.getElementById('dragCd').value;
        const frontalAreaM2 = document.getElementById('frontalAreaM2').value;
        const rollResCoeff = document.getElementById('rollResCoeff').value;
        const roadGradePct = document.getElementById('roadGradePct').value;
        const wheelRadiusM = document.getElementById('wheelRadiusM').value;
        const corrStandard = document.getElementById('corrStandard').value;

        const params = new URLSearchParams();
        params.append('weightKg', weightKg);
        params.append('tireSize', tireSize);
        params.append('humidityPct', humidityPct);
        params.append('pressureHpa', pressureHpa);
        params.append('unitsMetric', units === 'metric' ? '1' : '0');
        params.append('finalDriveRatio', finalDriveRatio);
        params.append('gearRatio', gearRatio);
        params.append('drivetrainLossPct', drivetrainLossPct);
        params.append('driveType', driveType);
        params.append('dragCd', dragCd);
        params.append('frontalAreaM2', frontalAreaM2);
        params.append('rollResCoeff', rollResCoeff);
        params.append('roadGradePct', roadGradePct);
        params.append('wheelRadiusM', wheelRadiusM);
        params.append('corrStandard', corrStandard);

        document.getElementById('status').textContent = 'Saving...';
        const res = await fetch('/api/settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: params.toString()
        });
        const j = await res.json();
        document.getElementById('status').textContent = j.ok ? 'Saved. Redirecting...' : 'Save failed.';
        if (j.ok) setTimeout(() => location.href = '/', 400);
      }

      document.getElementById('btnSave').addEventListener('click', saveSettings);
      loadSettings().catch(() => {
        document.getElementById('status').textContent = 'Could not load settings.';
      });
    </script>
  </body>
</html>
)HTML";

static void handleRoot() {
  httpServer.send_P(200, "text/html", kHomeHtml);
}

static String makeDummyPayload(uint32_t t_ms) {
  // Deterministic dummy waveforms so updates are visible.
  float phase = (t_ms % 40000) / 40000.0f;  // 40s cycle
  float speed_kmh_raw = 200.0f * phase;     // 0..200

  float rpm_raw = 800.0f + speed_kmh_raw * 35.0f; // 800..7000 (approx)
  if (rpm_raw > 7000.0f) rpm_raw = 7000.0f;

  float throttleRaw = 8.0f + 78.0f * phase; // 8..86%
  if (throttleRaw > 100.0f) throttleRaw = 100.0f;
  float fuelRateRaw = 1.8f + 0.020f * rpm_raw + 0.08f * throttleRaw; // rough L/h

  float speed_kmh = kalmanUpdate(g_kfSpeed, speed_kmh_raw);
  float rpm = kalmanUpdate(g_kfRpm, rpm_raw);
  float throttlePct = kalmanUpdate(g_kfThrottle, throttleRaw);
  float fuelRateLph = kalmanUpdate(g_kfFuelRate, fuelRateRaw);

  float torque = 180.0f + 120.0f * (1.0f - (rpm - 2500.0f) * (rpm - 2500.0f) / (2500.0f * 2500.0f));
  if (torque < 60.0f) torque = 60.0f;

  // Base power from OBDII torque/rpm and Kalman-smoothed signals.
  float hpCrank = torque * rpm / 7127.0f;
  if (hpCrank < 20.0f) hpCrank = 20.0f;
  if (hpCrank > 450.0f) hpCrank = 450.0f;

  const float kDriveLossFwdPct = 6.0f;
  const float kDriveLossRwdPct = 8.0f;
  const float kDriveLossAwdPct = 12.0f;
  float driveLossPct = kDriveLossFwdPct;
  if (g_driveType == "rwd") driveLossPct = kDriveLossRwdPct;
  if (g_driveType == "awd") driveLossPct = kDriveLossAwdPct;

  float mechLossPct = g_gearboxLossPct + driveLossPct;
  if (mechLossPct < 0.0f) mechLossPct = 0.0f;
  if (mechLossPct > 45.0f) mechLossPct = 45.0f;
  float hpMechLoss = hpCrank * (mechLossPct / 100.0f);

  const float speedMps = speed_kmh / 3.6f;
  static float prevSpeedMps = 0.0f;
  const float dtS = 0.1f; // 10 Hz
  const float accelMps2 = (speedMps - prevSpeedMps) / dtS;
  prevSpeedMps = speedMps;
  const float tempC = 22.0f + 12.0f * phase;
  const float tempK = tempC + 273.15f;
  const float pressurePa = (isnan(g_pressureHpa) ? 1013.25f : g_pressureHpa) * 100.0f;
  const float relHum = isnan(g_humidityPct) ? 50.0f : g_humidityPct;
  const float esPa = 610.94f * expf((17.625f * tempC) / (tempC + 243.04f));
  const float pvPa = (relHum * 0.01f) * esPa;
  const float pdPa = pressurePa - pvPa;
  const float rho = (pdPa / (287.058f * tempK)) + (pvPa / (461.495f * tempK));
  float fAeroN = 0.5f * rho * g_dragCd * g_frontalAreaM2 * speedMps * speedMps;
  float pAeroW = fAeroN * speedMps;
  float fRollN = g_rollResCoeff * (g_weightKg * 9.81f);
  float pRollW = fRollN * speedMps;
  float hpAeroLoss = pAeroW / 745.7f;
  float hpRollLoss = pRollW / 745.7f;

  // Slope correction (proxy until IMU+GPS fusion is integrated): user road grade input.
  float slopeRad = atanf(g_roadGradePct / 100.0f);
  float fSlopeN = g_weightKg * 9.81f * sinf(slopeRad);
  float pSlopeW = fSlopeN * speedMps;
  float hpSlopeLoss = pSlopeW / 745.7f;

  // Pro physics path: engine force = m*a + drag + rolling + slope.
  float fNetN = g_weightKg * accelMps2;
  float fEngineN = fNetN + fAeroN + fRollN + fSlopeN;
  if (fEngineN < 0.0f) fEngineN = 0.0f;
  float torqueFromForceNm = fEngineN * g_wheelRadiusM;
  float omegaRadS = rpm * (2.0f * PI / 60.0f);
  float hpFromForce = (torqueFromForceNm * omegaRadS) / 745.7f;
  if (hpFromForce > 5.0f) hpCrank = 0.5f * hpCrank + 0.5f * hpFromForce;

  float hpLossTotal = hpMechLoss + hpAeroLoss + hpRollLoss + hpSlopeLoss;
  if (hpLossTotal > hpCrank - 1.0f) hpLossTotal = hpCrank - 1.0f;
  if (hpLossTotal < 0.0f) hpLossTotal = 0.0f;
  float hpWheel = hpCrank - hpLossTotal;
  if (hpWheel < 0.0f) hpWheel = 0.0f;

  float hp = hpCrank;
  float airIntakeC = tempC;                                // from OBDII IAT
  float engineOilC = 78.0f + 24.0f * (rpm / 7000.0f);      // ~78..102 C

  // Correction factor K for DIN/SAE power correction.
  const float pKpa = pressurePa / 1000.0f;
  float kDin = (101.3f / pKpa) * sqrtf(tempK / 293.15f);
  float kSae = powf(99.0f / pKpa, 0.7f) * powf(tempK / 298.0f, 1.2f);
  if (kDin < 0.85f) kDin = 0.85f; if (kDin > 1.20f) kDin = 1.20f;
  if (kSae < 0.85f) kSae = 0.85f; if (kSae > 1.20f) kSae = 1.20f;
  float corrK = (g_corrStandard == "sae") ? kSae : kDin;
  float hpCorrected = hpCrank * corrK;

  // Basic predictive/anomaly placeholder for online diagnostics.
  float hpExpected = (throttlePct / 100.0f) * (0.065f * rpm);
  String anomaly = "none";
  if (throttlePct > 65.0f && hpCrank < hpExpected * 0.75f) anomaly = "power_drop";
  if (throttlePct > 55.0f && fuelRateLph < 3.0f) anomaly = "fuel_rate_low";

  char buf[920];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"live\",\"t_ms\":%lu,\"speed_kmh\":%.1f,\"rpm\":%.0f,\"hp\":%.0f,\"torque_nm\":%.0f,\"hp_wheel\":%.1f,\"hp_crank\":%.1f,\"hp_corrected\":%.1f,\"corr_factor_k\":%.3f,\"corr_std\":\"%s\",\"hp_expected\":%.1f,\"anomaly\":\"%s\",\"hp_loss_total\":%.1f,\"hp_loss_aero\":%.1f,\"hp_loss_roll\":%.1f,\"hp_loss_slope\":%.1f,\"loss_gearbox_pct\":%.1f,\"drive_type\":\"%s\",\"throttle_pct\":%.1f,\"fuel_rate_lph\":%.2f,\"air_intake_c\":%.1f,\"engine_oil_c\":%.1f,\"air_density\":%.4f,\"pressure_hpa\":%.1f,\"humidity_pct\":%.1f,\"setup_ok\":%s,\"missing_fields\":%s}",
           (unsigned long)t_ms, speed_kmh, rpm, hp, torque, hpWheel, hpCrank, hpCorrected, corrK, g_corrStandard.c_str(), hpExpected, anomaly.c_str(), hpLossTotal, hpAeroLoss, hpRollLoss, hpSlopeLoss, g_gearboxLossPct, g_driveType.c_str(), throttlePct, fuelRateLph, airIntakeC, engineOilC, rho, pressurePa / 100.0f, relHum,
           g_setupOk ? "true" : "false",
           g_missingFieldsJson.c_str());
  return String(buf);
}

static void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  (void)payload;
  (void)length;
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] client %u connected\n", num);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] client %u disconnected\n", num);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("DynoTrack X - skeleton starting...");

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false); // keep WiFi radio responsive for AP + WS traffic

  if (kUseDummyObd) {
    Serial.println("OBDII DUMMY mode enabled. Skipping real OBDII WiFi connect.");
  } else {
    Serial.print("Connecting to OBDII SSID: ");
    Serial.println(kObdSsid);
    WiFi.begin(kObdSsid);

    const uint32_t kObdConnectTimeoutMs = 12000;
    unsigned long connectStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - connectStart < kObdConnectTimeoutMs) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("OBDII connected. STA IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("OBDII WiFi connect timeout. Continuing with AP mode active.");
    }
  }

  IPAddress apIP(192, 168, 4, 1);
  IPAddress gw(192, 168, 4, 1);
  IPAddress nm(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gw, nm);

  const int kApChannel = 6;
  const bool kApHidden = false;
  const int kApMaxConnections = 4;
  bool ok = WiFi.softAP(kApSsid, nullptr, kApChannel, kApHidden, kApMaxConnections); // open AP
  if (!ok) Serial.println("Failed to start WiFi AP");

  Serial.print("AP SSID: ");
  Serial.println(kApSsid);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  loadSettings();

  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/settings", HTTP_GET, []() {
    httpServer.send_P(200, "text/html", kSettingsHtml);
  });

  httpServer.on("/api/settings", HTTP_GET, []() {
    httpServer.send(200, "application/json", jsonApiSettings());
  });

  httpServer.on("/api/settings", HTTP_POST, []() {
    // Expect x-www-form-urlencoded params
    if (!httpServer.hasArg("weightKg") || !httpServer.hasArg("tireSize")) {
      httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing mandatory fields\"}");
      return;
    }

    g_weightKg = httpServer.arg("weightKg").toFloat();
    g_tireSize = httpServer.arg("tireSize");
    g_tireSize.trim();

    // Optional fields (empty => keep NaN)
    if (httpServer.hasArg("humidityPct") && httpServer.arg("humidityPct").length() > 0) {
      g_humidityPct = httpServer.arg("humidityPct").toFloat();
    } else {
      g_humidityPct = NAN;
    }

    if (httpServer.hasArg("pressureHpa") && httpServer.arg("pressureHpa").length() > 0) {
      g_pressureHpa = httpServer.arg("pressureHpa").toFloat();
    } else {
      g_pressureHpa = NAN;
    }

    if (httpServer.hasArg("unitsMetric")) {
      g_unitsMetric = (httpServer.arg("unitsMetric") == "1");
    } else {
      g_unitsMetric = true;
    }

    if (httpServer.hasArg("finalDriveRatio") && httpServer.arg("finalDriveRatio").length() > 0) {
      g_finalDriveRatio = httpServer.arg("finalDriveRatio").toFloat();
    }
    if (httpServer.hasArg("gearRatio") && httpServer.arg("gearRatio").length() > 0) {
      g_gearRatio = httpServer.arg("gearRatio").toFloat();
    }
    if (httpServer.hasArg("drivetrainLossPct") && httpServer.arg("drivetrainLossPct").length() > 0) {
      g_gearboxLossPct = httpServer.arg("drivetrainLossPct").toFloat();
    }
    if (httpServer.hasArg("driveType") && httpServer.arg("driveType").length() > 0) {
      g_driveType = httpServer.arg("driveType");
      g_driveType.toLowerCase();
      if (g_driveType != "fwd" && g_driveType != "rwd" && g_driveType != "awd") g_driveType = "fwd";
    }
    if (httpServer.hasArg("dragCd") && httpServer.arg("dragCd").length() > 0) {
      g_dragCd = httpServer.arg("dragCd").toFloat();
    }
    if (httpServer.hasArg("frontalAreaM2") && httpServer.arg("frontalAreaM2").length() > 0) {
      g_frontalAreaM2 = httpServer.arg("frontalAreaM2").toFloat();
    }
    if (httpServer.hasArg("rollResCoeff") && httpServer.arg("rollResCoeff").length() > 0) {
      g_rollResCoeff = httpServer.arg("rollResCoeff").toFloat();
    }
    if (httpServer.hasArg("roadGradePct") && httpServer.arg("roadGradePct").length() > 0) {
      g_roadGradePct = httpServer.arg("roadGradePct").toFloat();
    }
    if (httpServer.hasArg("wheelRadiusM") && httpServer.arg("wheelRadiusM").length() > 0) {
      g_wheelRadiusM = httpServer.arg("wheelRadiusM").toFloat();
    }
    if (httpServer.hasArg("corrStandard") && httpServer.arg("corrStandard").length() > 0) {
      g_corrStandard = httpServer.arg("corrStandard");
      g_corrStandard.toLowerCase();
      if (g_corrStandard != "din" && g_corrStandard != "sae") g_corrStandard = "din";
    }

    if (!parseMandatoryFields()) {
      recomputeSetupState();
      httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid mandatory fields\"}");
      return;
    }

    prefs.begin("dyntx", false);
    prefs.putFloat("weightKg", g_weightKg);
    prefs.putString("tireSize", g_tireSize);
    if (!isnan(g_humidityPct)) prefs.putFloat("humidityPct", g_humidityPct);
    if (!isnan(g_pressureHpa)) prefs.putFloat("pressureHpa", g_pressureHpa);
    prefs.putBool("unitsMetric", g_unitsMetric);
    prefs.putFloat("finalDrive", g_finalDriveRatio);
    prefs.putFloat("gearRatio", g_gearRatio);
    prefs.putFloat("gbLossPct", g_gearboxLossPct);
    prefs.putString("driveType", g_driveType);
    prefs.putFloat("dragCd", g_dragCd);
    prefs.putFloat("frAreaM2", g_frontalAreaM2);
    prefs.putFloat("rollRes", g_rollResCoeff);
    prefs.putFloat("gradePct", g_roadGradePct);
    prefs.putFloat("wheelRadM", g_wheelRadiusM);
    prefs.putString("corrStd", g_corrStandard);
    prefs.end();

    recomputeSetupState();

    httpServer.send(200, "application/json", "{\"ok\":true}");
  });

  httpServer.onNotFound([]() {
    httpServer.send(404, "text/plain", "Not found");
  });
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(wsEvent);

  Serial.println("HTTP server started, WS started.");
}

void loop() {
  wsServer.loop();
  httpServer.handleClient();

  static unsigned long lastMs = 0;
  static uint32_t tMs = 0;
  const uint32_t periodMs = 100; // 10 Hz

  unsigned long nowMs = millis();
  if (nowMs - lastMs >= periodMs) {
    lastMs = nowMs;
    tMs += periodMs;

    if (wsServer.connectedClients() > 0) {
      String payload = makeDummyPayload(tMs);
      wsServer.broadcastTXT(payload);
    }
  }
}