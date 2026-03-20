#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>

// DynoTrack X - day-1 skeleton (sync stack)
// - ESP32-S3 exposes WiFi AP: "DynoTrack X" (open)
// - HTTP server on :80 serves a minimal dashboard at "/"
// - WebSocket server on :81 pushes dummy live JSON at 10 Hz

static const char* kApSsid = "DynoTrack X";
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

static bool g_setupOk = false;
static String g_missingFieldsJson = "[]";

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
  prefs.end();

  recomputeSetupState();
}

static bool parseMandatoryFields() {
  if (g_weightKg <= 0.1f) return false;
  if (g_tireSize.length() < 3) return false;
  return true;
}

static String jsonApiSettings() {
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"setup_ok\":%s,\"weightKg\":%.2f,\"tireSize\":\"%s\",\"humidityPct\":%.1f,\"pressureHpa\":%.1f,\"unitsMetric\":%s,\"missing_fields\":%s}",
           g_setupOk ? "true" : "false",
           (double)g_weightKg,
           g_tireSize.c_str(),
           isnan(g_humidityPct) ? 0.0 : (double)g_humidityPct,
           isnan(g_pressureHpa) ? 0.0 : (double)g_pressureHpa,
           g_unitsMetric ? "true" : "false",
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
        background: #0b0f14;
        color: #e9eef5;
      }
      header {
        padding: 16px 18px;
        border-bottom: 1px solid rgba(255,255,255,0.08);
        display: flex;
        align-items: baseline;
        justify-content: space-between;
        gap: 12px;
      }
      header .title { font-weight: 800; letter-spacing: 0.3px; }
      header .sub { font-size: 12px; opacity: 0.8; }
      .headerRight {
        display: flex;
        align-items: center;
        gap: 12px;
      }
      .settingsLink {
        font-size: 12px;
        opacity: 0.95;
        font-weight: 800;
        text-decoration: none;
        color: #b9f6ca;
        border: 1px solid rgba(255,255,255,0.16);
        background: rgba(255,255,255,0.05);
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
        border: 1px solid rgba(255,255,255,0.10);
        border-radius: 14px;
        padding: 14px 14px 16px;
        background: rgba(255,255,255,0.03);
      }
      .label { font-size: 12px; opacity: 0.75; margin-bottom: 6px; }
      .value {
        font-size: 34px;
        font-weight: 800;
        letter-spacing: 0.2px;
      }
      .unit { font-size: 12px; opacity: 0.8; font-weight: 600; margin-left: 6px;}
      .status {
        margin-top: 14px;
        font-size: 12px;
        opacity: 0.8;
        display: flex;
        gap: 10px;
        flex-wrap: wrap;
      }
      .pill {
        border: 1px solid rgba(255,255,255,0.15);
        background: rgba(255,255,255,0.05);
        padding: 6px 10px;
        border-radius: 999px;
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
    </style>
  </head>
  <body>
    <header>
      <div>
        <div class="title">DynoTrack X</div>
        <div class="sub">WebSocket live skeleton (10 Hz dummy data)</div>
      </div>
      <div class="headerRight">
        <a class="settingsLink" href="/settings">Settings</a>
        <div id="wsState" class="sub">WS: connecting...</div>
      </div>
    </header>

    <div class="wrap">
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
          <div class="label">RPM</div>
          <div class="value"><span id="rpm">0</span><span class="unit">rpm</span></div>
        </div>

        <div class="card">
          <div class="label">HP</div>
          <div class="value"><span id="hp">0</span><span class="unit">hp</span></div>
        </div>

        <div class="card">
          <div class="label">Torque</div>
          <div class="value"><span id="torque">0</span><span class="unit">Nm</span></div>
        </div>
      </div>

      <div class="status">
        <div class="pill">Dummy push: 10 Hz</div>
        <div class="pill" id="tInfo">t_ms: 0</div>
      </div>
    </div>

    <script>
      const wsUrl = 'ws://' + location.hostname + ':81/';
      const wsState = document.getElementById('wsState');
      const elSpeed = document.getElementById('speed');
      const elRpm = document.getElementById('rpm');
      const elHp = document.getElementById('hp');
      const elTorque = document.getElementById('torque');
      const elTInfo = document.getElementById('tInfo');

      const setupBanner = document.getElementById('setupBanner');
      const setupMsg = document.getElementById('setupMsg');

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

      const ws = new WebSocket(wsUrl);
      ws.onopen = () => wsState.textContent = 'WS: connected';
      ws.onclose = () => wsState.textContent = 'WS: disconnected';
      ws.onerror = () => wsState.textContent = 'WS: error';

      ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (!msg || msg.type !== 'live') return;
        updateSetup(msg);
        elSpeed.textContent = Number(msg.speed_kmh).toFixed(0);
        elRpm.textContent = Number(msg.rpm).toFixed(0);
        elHp.textContent = Number(msg.hp).toFixed(0);
        elTorque.textContent = Number(msg.torque_nm).toFixed(0);
        elTInfo.textContent = 't_ms: ' + msg.t_ms;
      };
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
        background: #0b0f14;
        color: #e9eef5;
      }
      .wrap { padding: 18px; max-width: 520px; margin: 0 auto; }
      h1 { font-size: 18px; margin: 0 0 10px; }
      .card {
        border: 1px solid rgba(255,255,255,0.10);
        border-radius: 14px;
        padding: 14px;
        background: rgba(255,255,255,0.03);
      }
      .row { margin: 12px 0; }
      label { display:block; font-size: 12px; opacity: 0.75; margin-bottom: 6px; }
      input, select {
        width: 100%;
        padding: 10px 12px;
        border-radius: 12px;
        border: 1px solid rgba(255,255,255,0.12);
        background: rgba(0,0,0,0.2);
        color: #e9eef5;
        font-size: 14px;
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
      .msg { font-size: 12px; opacity: 0.8; margin-top: 10px; }
      a { color: #b9f6ca; }
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

        <button id="btnSave">Save</button>
        <div class="msg" id="status">Loading saved settings...</div>
        <div class="msg"><a href="/">Back to Home</a></div>
      </div>
    </div>

    <script>
      async function loadSettings() {
        const res = await fetch('/api/settings');
        const j = await res.json();
        document.getElementById('weightKg').value = j.weightKg ? j.weightKg : '';
        document.getElementById('tireSize').value = j.tireSize ? j.tireSize : '';
        document.getElementById('humidityPct').value = (j.humidityPct && j.humidityPct > 0) ? j.humidityPct : '';
        document.getElementById('pressureHpa').value = (j.pressureHpa && j.pressureHpa > 0) ? j.pressureHpa : '';
        document.getElementById('units').value = j.unitsMetric ? 'metric' : 'imperial';
        document.getElementById('status').textContent = j.setup_ok ? 'Setup OK' : 'Setup not complete yet';
      }

      async function saveSettings() {
        const weightKg = document.getElementById('weightKg').value;
        const tireSize = document.getElementById('tireSize').value;
        const humidityPct = document.getElementById('humidityPct').value;
        const pressureHpa = document.getElementById('pressureHpa').value;
        const units = document.getElementById('units').value;

        const params = new URLSearchParams();
        params.append('weightKg', weightKg);
        params.append('tireSize', tireSize);
        params.append('humidityPct', humidityPct);
        params.append('pressureHpa', pressureHpa);
        params.append('unitsMetric', units === 'metric' ? '1' : '0');

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
  float speed_kmh = 200.0f * phase;         // 0..200

  float rpm = 800.0f + speed_kmh * 35.0f; // 800..7000 (approx)
  if (rpm > 7000.0f) rpm = 7000.0f;

  float torque = 180.0f + 120.0f * (1.0f - (rpm - 2500.0f) * (rpm - 2500.0f) / (2500.0f * 2500.0f));
  if (torque < 60.0f) torque = 60.0f;

  float hp = torque * rpm / 7127.0f;       // rough visual estimate
  if (hp < 20.0f) hp = 20.0f;
  if (hp > 450.0f) hp = 450.0f;

  char buf[260];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"live\",\"t_ms\":%lu,\"speed_kmh\":%.1f,\"rpm\":%.0f,\"hp\":%.0f,\"torque_nm\":%.0f,\"setup_ok\":%s,\"missing_fields\":%s}",
           (unsigned long)t_ms, speed_kmh, rpm, hp, torque,
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

  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gw(192, 168, 4, 1);
  IPAddress nm(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gw, nm);

  bool ok = WiFi.softAP(kApSsid); // open AP (no password)
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