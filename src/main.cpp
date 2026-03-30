#include <Arduino.h>
#include <WiFi.h>
/* Larger WS send queue: phones sometimes ACK slowly during long dyno runs; default 32 can drop bursts. */
#if !defined(WS_MAX_QUEUED_MESSAGES)
#define WS_MAX_QUEUED_MESSAGES 64
#endif
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <math.h>
#include "esp_task_wdt.h"
#include "logo_png.h"

// Optional external W25Q64 (8 MByte SPI NOR) for logs / run history (wire in firmware later).
// Suggested ESP32-S3 DevKit wiring — keep UART pins free for GPS module:
//   SPI: CS=GPIO10, MOSI=GPIO11, MISO=GPIO12, SCK=GPIO13 (HSPI/FSPI-capable GPIOs; avoid strapping pins 0,3,45,46).
//   GPS UART (example): RX=GPIO4, TX=GPIO5 (UART1) or RX=GPIO44, TX=GPIO43 — avoid USB/JTAG pins if in use.

// DynoTrack X - day-1 skeleton (sync stack)
// - ESP32-S3 first tries to join OBDII WiFi STA: "WIFI_OBDII."
// - ESP32-S3 also exposes WiFi AP: "DynoTrack-X" (open)
// - HTTP on :80 serves dashboard + settings; WebSocket live stream on same port at "/ws"
//   (avoids phones / captive portals blocking non-:80 TCP ports such as :81)

static const char* kObdSsid = "WIFI_OBDII.";
static const char* kApSsid = "DynoTrack-X";
static const bool kUseDummyObd = true; // keep true until physical OBDII arrives
static const uint16_t kHttpPort = 80;

static AsyncWebServer httpServer(kHttpPort);
static AsyncWebSocket wsLive("/ws");

static Preferences prefs;

static float g_weightKg = 0.0f;
static String g_tireSize = "";
static float g_humidityPct = NAN;
static float g_pressureHpa = NAN;
/** Optional °C entered in Setup for notes only; not used in physics / correction. */
static float g_ambientTempNoteC = NAN;
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
static float g_redlineRpm = 6500.0f;
static String g_powerUnitPref = "hp";
static bool g_selfCalEnabled = true;
static float g_selfCalConfidence = 0.0f;
static bool g_selfCalLocked = false;
static bool g_selfCalSaved = false;
static bool g_prefsSaveSelfCalPending = false;
static float g_signalNoisePct = 0.0f;
static float g_signalDriftPct = 0.0f;
static float g_signalResolutionPct = 100.0f;
static uint32_t g_wsPeriodMs = kUseDummyObd ? 200u : 130u;

static bool g_setupOk = false;
static String g_missingFieldsJson = "[]";
static bool g_coastCalValid = false;
static float g_coastCalConfidence = 0.0f;
static String g_coastCalReason = "repeat procedure";
static bool g_coastBypass = false;
static float g_autoArmSpeedKmh = 15.0f;
/** When true (default), choosing a measurement mode arms immediately; when false, user must press START RUN first (same start triggers either way). */
static bool g_measurementAutoArm = true;
static float g_batteryVoltFilt = NAN;

// Battery monitor via divider: VBAT -> 100k -> ADC -> 27k -> GND.
// Internal pack SOC is estimated from voltage (firmware); UI shows only % and status, not raw V.
// Above ~9.5 V on the sense pin => profile "external" (no % for the pack).
static const int kBatterySensePin = 1;
static const float kBatteryDividerRTop = 100000.0f;
static const float kBatteryDividerRBottom = 27000.0f;
static const float kBatteryLi2sMinV = 6.0f;
static const float kBatteryLi2sMaxV = 8.4f;
// When true, WebSocket cycles demo battery_pct / state for the unit pack. Set false when ADC is wired.
static const bool kBatteryDemoPayload = true;

struct Kalman1D {
  float x;
  float p;
  float q;
  float r;
  bool initialized;
};

static Kalman1D g_kfRpm = {0, 1, 0.6f, 20.0f, false};
static Kalman1D g_kfSpeed = {0, 1, 0.2f, 2.0f, false};
static Kalman1D g_kfGpsSpeed = {0, 1, 0.25f, 2.8f, false};
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

static float readBatteryVoltageV() {
  uint32_t mvSum = 0;
  const int samples = 8;
  for (int i = 0; i < samples; i++) {
    mvSum += (uint32_t)analogReadMilliVolts(kBatterySensePin);
    delayMicroseconds(250);
  }
  const float adcV = ((float)mvSum / (float)samples) / 1000.0f;
  const float scale = (kBatteryDividerRTop + kBatteryDividerRBottom) / kBatteryDividerRBottom;
  const float vBat = adcV * scale;
  if (vBat < 5.0f || vBat > 20.0f) return NAN;
  if (isnan(g_batteryVoltFilt)) g_batteryVoltFilt = vBat;
  else g_batteryVoltFilt = 0.82f * g_batteryVoltFilt + 0.18f * vBat;
  return g_batteryVoltFilt;
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
  g_ambientTempNoteC = prefs.getFloat("ambNoteC", NAN);

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
  g_redlineRpm = prefs.getFloat("redlineRpm", 6500.0f);
  g_powerUnitPref = prefs.getString("powerUnit", "hp");
  g_powerUnitPref.toLowerCase();
  if (g_powerUnitPref != "hp" && g_powerUnitPref != "kw") g_powerUnitPref = "hp";
  g_coastCalValid = prefs.getBool("coastValid", false);
  g_coastCalConfidence = prefs.getFloat("coastConf", 0.0f);
  g_coastCalReason = prefs.getString("coastReason", "repeat procedure");
  g_coastBypass = prefs.getBool("coastBypass", false);
  g_autoArmSpeedKmh = prefs.getFloat("autoArmKmh", 15.0f);
  if (g_autoArmSpeedKmh < 0.0f) g_autoArmSpeedKmh = 0.0f;
  if (g_autoArmSpeedKmh > 200.0f) g_autoArmSpeedKmh = 200.0f;
  g_measurementAutoArm = prefs.getBool("measAutoArm", true);
  prefs.end();

  recomputeSetupState();
}

static bool parseMandatoryFields() {
  if (g_weightKg <= 0.1f) return false;
  if (g_tireSize.length() < 3) return false;
  return true;
}

static String jsonApiSettings() {
  char ambJson[28];
  if (isnan(g_ambientTempNoteC)) {
    strncpy(ambJson, "null", sizeof(ambJson));
    ambJson[sizeof(ambJson) - 1] = '\0';
  } else {
    snprintf(ambJson, sizeof(ambJson), "%.1f", (double)g_ambientTempNoteC);
  }
  char buf[1024];
  snprintf(buf, sizeof(buf),
           "{\"setup_ok\":%s,\"weightKg\":%.2f,\"tireSize\":\"%s\",\"humidityPct\":%.1f,\"pressureHpa\":%.1f,\"ambientTempNoteC\":%s,\"unitsMetric\":%s,\"finalDriveRatio\":%.3f,\"gearRatio\":%.3f,\"drivetrainLossPct\":%.1f,\"dragCd\":%.3f,\"frontalAreaM2\":%.3f,\"rollResCoeff\":%.4f,\"roadGradePct\":%.2f,\"wheelRadiusM\":%.4f,\"driveType\":\"%s\",\"corrStandard\":\"%s\",\"redlineRpm\":%.0f,\"powerUnit\":\"%s\",\"autoArmKmh\":%.1f,\"measurementAutoArm\":%s,\"coastCalValid\":%s,\"coastCalConf\":%.1f,\"coastCalReason\":\"%s\",\"coastBypass\":%s,\"missing_fields\":%s}",
           g_setupOk ? "true" : "false",
           (double)g_weightKg,
           g_tireSize.c_str(),
           isnan(g_humidityPct) ? 0.0 : (double)g_humidityPct,
           isnan(g_pressureHpa) ? 0.0 : (double)g_pressureHpa,
           ambJson,
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
           (double)g_redlineRpm,
           g_powerUnitPref.c_str(),
           (double)g_autoArmSpeedKmh,
           g_measurementAutoArm ? "true" : "false",
           g_coastCalValid ? "true" : "false",
           (double)g_coastCalConfidence,
           g_coastCalReason.c_str(),
           g_coastBypass ? "true" : "false",
           g_missingFieldsJson.c_str());
  return String(buf);
}

static const char kHomeHtml[] PROGMEM = R"HTML(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1"/>
    <meta name="apple-mobile-web-app-capable" content="yes"/>
    <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent"/>
    <meta name="theme-color" content="#000000"/>
    <title>DynoTrack X</title>
    <style>
      :root { color-scheme: dark; }
      html { height: 100%; }
      body {
        margin: 0;
        min-height: 100%;
        overflow-x: hidden;
        overflow-y: auto;
        -webkit-overflow-scrolling: touch;
        font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
        background: #000000;
        color: #ffffff;
        -webkit-text-size-adjust: 100%;
        touch-action: manipulation;
      }
      header {
        padding: 12px 18px 16px;
        border-bottom: 1px solid rgba(255,255,255,0.16);
        display: grid;
        grid-template-columns: minmax(0, 1fr) auto minmax(240px, 420px);
        gap: 12px 14px;
        align-items: center;
      }
      .headerLeft {
        display: flex;
        flex-direction: column;
        align-items: flex-start;
        justify-content: center;
        min-width: 0;
      }
      .headerCenter {
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
        min-width: 0;
        transform: translateX(-66.67%);
      }
      .headerRight {
        display: flex;
        flex-direction: column;
        flex-wrap: nowrap;
        justify-content: center;
        align-items: flex-end;
        gap: 8px;
        min-width: 0;
      }
      @media (max-width: 720px) {
        header {
          grid-template-columns: 1fr;
        }
        .headerLeft { justify-self: start; }
        .headerCenter { justify-self: center; }
        .headerRight {
          align-items: center;
          width: 100%;
        }
        .brandLogo { max-width: min(100%, 240px); }
      }
      /*
        Phone / touch narrow UI: toggled via html.dt-phone-header-badge (JS) so desktop mouse
        keeps the original .headerCenter transform; phones reset the column and shift the badge
        right by one label width (same idea as run-cue translate %).
      */
      html.dt-phone-header-badge .headerCenter {
        transform: none;
      }
      html.dt-phone-header-badge .headerCenter #wsState.wsBadge {
        transform: translate3d(100%, 0, 0);
      }
      .brandLogo {
        display: block;
        margin: 0;
        max-width: min(100%, 260px);
        width: 100%;
        height: auto;
      }
      .wsBadge {
        display: inline-block;
        font-size: 12px;
        font-weight: 700;
        opacity: 1;
        padding: 6px 10px;
        border-radius: 999px;
        border: 1pxf solid rgba(255,255,255,0.35);
        white-space: nowrap;
        flex-shrink: 0;
      }
      .wsBadge.ws-disconnected {
        color: #fff;
        background: rgba(180,40,40,0.95);
        border-color: rgba(255,120,120,0.85);
      }
      .wsBadge.ws-connecting {
        color: #000;
        background: rgba(255,214,70,0.98);
        border-color: rgba(255,230,120,0.95);
      }
      .wsBadge.ws-connected {
        color: #000;
        background: rgba(0,220,120,0.95);
        border-color: rgba(0,255,160,0.75);
      }
      .settingsLink {
        display: inline-block;
        font-size: 14px;
        opacity: 1;
        font-weight: 800;
        text-decoration: none;
        color: #00ff00;
        text-shadow: 0 0 10px #00ff00, 0 0 20px #00ff00, 0 0 30px #00ff00;
        border: 1px solid #00ff00;
        background: #000000;
        padding: 8px 12px;
        border-radius: 10px;
      }
      .settingsBottomLeft {
        position: fixed;
        left: 14px;
        bottom: calc(14px + env(safe-area-inset-bottom, 0px));
        z-index: 9999;
        pointer-events: auto;
      }
      .calButtons {
        display: flex;
        gap: 10px;
        align-items: center;
      }
      .calButtons button {
        font-size: 12px;
        padding: 6px 10px;
        border-radius: 8px;
        border: none;
        background: #00ff88;
        color: #00110a;
        font-weight: 700;
        cursor: pointer;
      }
      .batteryBlock {
        display: inline-flex;
        flex-direction: column;
        align-items: flex-end;
        gap: 5px;
        max-width: 220px;
      }
      .batteryBlockTitle {
        font-size: 10px;
        font-weight: 800;
        letter-spacing: 0.04em;
        color: rgba(210, 235, 255, 0.92);
        text-align: right;
        width: 100%;
      }
      .batteryKeyLegend {
        font-size: 9px;
        font-weight: 600;
        color: rgba(200, 220, 240, 0.65);
        display: flex;
        flex-wrap: wrap;
        justify-content: flex-end;
        gap: 6px 10px;
        line-height: 1.2;
      }
      .batteryKeyLegend .bkDot {
        display: inline-block;
        width: 6px;
        height: 6px;
        border-radius: 50%;
        margin-right: 3px;
        vertical-align: middle;
        position: relative;
        top: -1px;
      }
      .batteryKeyLegend .bkDot--g { background: #00ff88; }
      .batteryKeyLegend .bkDot--y { background: #ffd84d; }
      .batteryKeyLegend .bkDot--r { background: #ff5f5f; }
      .batteryWidget {
        display: inline-flex;
        align-items: center;
        gap: 8px;
        padding: 6px 10px;
        border: 1px solid rgba(255,255,255,0.30);
        border-radius: 12px;
        background: rgba(255,255,255,0.08);
      }
      .batteryInfoCol {
        display: flex;
        flex-direction: column;
        gap: 4px;
        align-items: flex-end;
        min-width: 128px;
      }
      .batRow {
        font-size: 11px;
        line-height: 1.25;
        text-align: right;
        width: 100%;
      }
      .batRow .batLbl {
        color: rgba(170, 205, 255, 0.78);
        font-weight: 600;
        margin-right: 8px;
      }
      .batRow .batVal {
        color: #e8fff0;
        font-weight: 700;
        font-variant-numeric: tabular-nums;
      }
      .batteryIcon {
        position: relative;
        width: 34px;
        height: 16px;
        border: 2px solid #9ab0c8;
        border-radius: 4px;
        box-sizing: border-box;
      }
      .batteryIcon::after {
        content: "";
        position: absolute;
        right: -6px;
        top: 4px;
        width: 4px;
        height: 6px;
        border-radius: 1px;
        background: currentColor;
      }
      .batteryFill {
        position: absolute;
        left: 1px;
        top: 1px;
        bottom: 1px;
        width: 8%;
        border-radius: 2px;
        background: #9ab0c8;
        transition: width 160ms linear, background 160ms linear;
      }
      .batteryWidget.state-green .batteryIcon { border-color: #00ff88; color: #00ff88; }
      .batteryWidget.state-green .batteryFill { background: #00ff88; }
      .batteryWidget.state-yellow .batteryIcon { border-color: #ffd84d; color: #ffd84d; }
      .batteryWidget.state-yellow .batteryFill { background: #ffd84d; }
      .batteryWidget.state-red .batteryIcon { border-color: #ff5f5f; color: #ff5f5f; }
      .batteryWidget.state-red .batteryFill { background: #ff5f5f; }
      .batteryWidget.state-unknown .batteryIcon { border-color: #9ab0c8; color: #9ab0c8; }
      .batteryWidget.state-unknown .batteryFill { background: #9ab0c8; }
      .batteryWidget.state-external .batteryIcon { border-color: #7eb8ff; color: #7eb8ff; }
      .batteryWidget.state-external .batteryFill { background: #7eb8ff; }
      .wrap { padding: 16px 18px 32px; box-sizing: border-box; }
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
      .controls { margin-top: 14px; display: flex; gap: 10px; flex-wrap: wrap; align-items: flex-end; }
      .measurementMenuBlock {
        display: flex;
        flex-direction: column;
        gap: 8px;
        min-width: 0;
        width: 100%;
      }
      .measurementMenuTitle {
        font-size: 11px;
        font-weight: 800;
        color: #c8d8ff;
        letter-spacing: 0.06em;
        text-transform: uppercase;
      }
      .btn {
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(255,255,255,0.12);
        color: #fff;
        border-radius: 10px;
        padding: 10px 14px;
        font-weight: 800;
      }
      .btn:disabled {
        opacity: 0.45;
        cursor: not-allowed;
      }
      /* Same green family as header "connected" badge + battery accent */
      .btn.btnGreen {
        border: 1px solid rgba(0, 255, 160, 0.75);
        background: rgba(0, 220, 120, 0.95);
        color: #00180f;
        box-shadow: 0 1px 0 rgba(255,255,255,0.25) inset;
        transition: transform 0.08s ease, filter 0.12s ease, box-shadow 0.12s ease;
        -webkit-tap-highlight-color: transparent;
      }
      .btn.btnGreen:hover:not(:disabled) {
        filter: brightness(1.05);
      }
      .btn.btnGreen:active:not(:disabled):not(#btnStartRun),
      .btn.btnGreen.btnStartRun--pressed:not(:disabled):not(#btnStartRun) {
        transform: scale(0.98);
        filter: none;
        border-width: 2px;
        border-color: rgba(0, 90, 55, 0.95);
        background: rgba(0, 95, 52, 1);
        color: #e8fff4;
        box-shadow: inset 0 3px 14px rgba(0, 0, 0, 0.45);
      }
      /* START RUN: #id + !important so .btnStartRun--armed never hides the “pressed” dark fill */
      button#btnStartRun.btn.btnGreen {
        margin-top: 8px;
        font-weight: 900;
      }
      button#btnStartRun.btn.btnGreen:hover:not(:disabled) {
        filter: none;
      }
      button#btnStartRun.btn.btnGreen.btnStartRun--invalid:not(:disabled) {
        background: rgba(90, 45, 0, 0.98) !important;
        border-color: rgba(255, 190, 90, 0.95) !important;
        color: #fff8e8 !important;
        box-shadow: inset 0 2px 10px rgba(0, 0, 0, 0.35) !important;
      }
      /* Armed, finger up: medium green — clearly darker than idle */
      button#btnStartRun.btn.btnGreen.btnStartRun--armed:not(:disabled):not(.btnStartRun--running) {
        background: rgb(0, 42, 26) !important;
        border-color: rgba(0, 220, 160, 0.88) !important;
        color: #e8fff4 !important;
        box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.12) !important;
      }
      button#btnStartRun.btn.btnGreen.btnStartRun--running:not(:disabled) {
        background: rgba(0, 190, 230, 0.92) !important;
        border-color: rgba(120, 230, 255, 0.9) !important;
        color: #001820 !important;
      }
      button#btnStartRun.btn.btnGreen:active:not(:disabled),
      button#btnStartRun.btn.btnGreen.btnStartRun--pressed:not(:disabled) {
        transform: scale(0.96);
        filter: none !important;
        border-width: 2px !important;
        border-color: rgb(0, 52, 32) !important;
        background: rgb(0, 42, 26) !important;
        color: #e8fff4 !important;
        box-shadow: inset 0 5px 20px rgba(0, 0, 0, 0.58) !important;
      }
      button#btnStartRun.btn.btnGreen.btnStartRun--invalid:active:not(:disabled),
      button#btnStartRun.btn.btnGreen.btnStartRun--invalid.btnStartRun--pressed:not(:disabled) {
        background: rgba(65, 30, 0, 1) !important;
        border-color: rgba(255, 200, 100, 0.95) !important;
        color: #fff8e8 !important;
        box-shadow: inset 0 5px 18px rgba(0, 0, 0, 0.45) !important;
      }
      .chartCard, .runsCard, .reportCard {
        margin-top: 14px;
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 12px;
        background: rgba(255,255,255,0.06);
      }
      #dynoGraphCard {
        min-width: 0;
      }
      #dynoCanvas {
        display: block;
        width: 100%;
        height: 260px;
        background: #050505;
        border-radius: 10px;
      }
      @media (max-width: 720px) {
        #dynoGraphCard {
          padding: 10px 10px 12px;
        }
        #dynoCanvas {
          height: min(52vw, 280px);
          min-height: 200px;
        }
      }
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
      .subNavDashBack {
        margin-top: 10px;
        margin-bottom: 4px;
        display: none;
        align-items: center;
        gap: 8px;
        flex-wrap: wrap;
      }
      .screenBtn {
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(255,255,255,0.10);
        color: #fff;
        border-radius: 10px;
        padding: 8px 12px;
        font-weight: 800;
      }
      .screenBtn.active { background: rgba(255,255,255,0.16); border-color: rgba(255,255,255,0.48); }
      #btnBackHome {
        background: rgba(255, 210, 60, 0.28);
        border-color: rgba(255, 215, 80, 0.85);
        color: #fff3b0;
        touch-action: manipulation;
      }
      .modeSelect {
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(255,255,255,0.12);
        color: #fff;
        border-radius: 10px;
        padding: 8px 10px;
        font-weight: 700;
      }
      /* Dark native picker on mobile where supported — avoids light-on-yellow option sheets */
      #measurementMode {
        color-scheme: dark;
      }
      /* Closed “pick a mode” state: dark text on amber — readable. Open list: dark rows + amber group labels. */
      #measurementMode.modeRequired {
        background: rgba(255, 200, 55, 0.92);
        color: #1a1400;
        border-color: rgba(160, 110, 0, 0.95);
      }
      #measurementMode option[value=""] {
        background: #2d2100;
        color: #ffe9a8;
        font-weight: 800;
      }
      .measurementModeSummary {
        display: block;
        margin-top: 8px;
        font-size: 12px;
        font-weight: 800;
        letter-spacing: 0.03em;
        color: rgba(180, 220, 255, 0.92);
      }
      .measurementModeSummary:empty {
        display: none;
      }
      .measurementCloseRow {
        display: none;
        flex-direction: row;
        flex-wrap: wrap;
        align-items: center;
        gap: 10px;
        margin-top: 10px;
      }
      .measurementCloseRow.visible {
        display: flex;
      }
      .measurementCloseHint {
        font-size: 11px;
        font-weight: 600;
        color: rgba(200, 220, 255, 0.78);
        max-width: 100%;
      }
      button.measurementCloseBtn {
        font-size: 12px;
        padding: 8px 14px;
        flex-shrink: 0;
      }
      /* Technical live pills (still updated in JS for logic; hidden on dashboard to reduce clutter while driving) */
      #homeStatusRow1,
      #homeStatusRow2,
      #homeStatusRow3 {
        display: none !important;
      }
      .trackModePanel {
        display: none;
        margin-top: 16px;
      }
      .trackModePanel.visible {
        display: block;
      }
      .trackModePanelHead {
        display: flex;
        flex-direction: row;
        align-items: center;
        justify-content: space-between;
        gap: 10px;
        flex-wrap: wrap;
        margin-bottom: 10px;
      }
      .trackModePanelHead .label {
        margin-bottom: 0;
      }
      .modeSelect option,
      #measurementMode option {
        background-color: #141414;
        color: #f0f0f0;
      }
      #measurementMode optgroup {
        background-color: #0a0a0a;
        color: #ffb020;
        font-weight: 800;
      }
      #measurementMode:disabled {
        opacity: 0.55;
        cursor: not-allowed;
      }
      .screenBlock { display: none; }
      .screenBlock.active { display: block; }
      /* Scroll past fixed dock so pills (e.g. Reason: …) sit above the menu, not hidden behind it */
      .screenBlock[data-screen="home"] {
        padding-bottom: calc(280px + env(safe-area-inset-bottom, 0px));
      }
      /* Dyno: tall help card above the dock — extra scroll so “how to drive” clears MEASUREMENT MENU */
      .screenBlock[data-screen="home"].dynoModeExtraScroll {
        padding-bottom: calc(460px + env(safe-area-inset-bottom, 0px));
      }
      .measurementLiveStatus {
        margin-top: 12px;
        margin-bottom: 8px;
        scroll-margin-bottom: min(300px, 42vh);
      }
      .measurementGrid {
        margin-top: 14px;
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 14px;
        min-height: min(320px, 48vh);
      }
      /* Hidden numeric cards keep element IDs for paintHomeCardsFromMsg(); gauges + dock are the main view */
      .screenBlock[data-screen="home"] .grid {
        display: none;
      }
      /* Scroll target when a run starts — keeps gauges clear of edges / fixed bottom bar */
      #analogGaugesAnchor {
        scroll-margin-top: 12px;
        scroll-margin-bottom: min(120px, 28vh);
      }
      .gaugeRow {
        margin-top: 10px;
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 12px;
      }
      .gaugeCard {
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 8px;
        background: rgba(255,255,255,0.06);
      }
      .gaugeCanvas {
        width: 100%;
        height: 180px;
        display: block;
      }
      .measureCard {
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 14px;
        background: rgba(255,255,255,0.06);
        text-align: center;
      }
      .gpsConfidenceBadge {
        margin-top: 10px;
        border: 1px solid rgba(255,255,255,0.28);
        border-radius: 14px;
        padding: 10px 12px;
        font-size: 13px;
        font-weight: 800;
        line-height: 1.35;
        background: rgba(255,255,255,0.08);
      }
      .measureValue { font-size: clamp(44px, 10vw, 88px); font-weight: 900; line-height: 1.0; }
      .measureLabel { font-size: 12px; opacity: 0.95; margin-bottom: 8px; }
      .liveBottomBar {
        position: fixed;
        left: 14px;
        right: 118px;
        bottom: calc(14px + env(safe-area-inset-bottom, 0px));
        z-index: 9998;
        display: flex;
        justify-content: center;
        pointer-events: none;
      }
      .liveBottomBarInner {
        pointer-events: auto;
        display: flex;
        flex-direction: column;
        align-items: stretch;
        gap: 10px;
        width: 100%;
        max-width: 520px;
      }
      .liveBottomBar .measurementMenuBlock {
        margin: 0;
        width: 100%;
        min-width: 0;
      }
      .liveActionsRow {
        display: flex;
        flex-direction: row;
        flex-wrap: wrap;
        align-items: flex-end;
        justify-content: center;
        gap: 8px;
      }
      .liveActionsRow .screenBtn {
        padding: 10px 12px;
        font-size: 11px;
      }
      .liveActionsRow .btn {
        font-size: 11px;
        font-weight: 800;
        padding: 10px 12px;
        letter-spacing: 0.02em;
      }
      .liveActionsRow button#btnStartRun.btn.btnGreen {
        margin-top: 0;
      }
      .projectsCard {
        margin-top: 14px;
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 12px;
        background: rgba(255,255,255,0.06);
      }
      .inlineInput {
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(255,255,255,0.10);
        color: #fff;
        border-radius: 10px;
        padding: 8px 10px;
        font-size: 12px;
        width: 4.2rem;
      }
      /* Measurement custom inputs: full-width readable panel (not cramped 4.2rem chips) */
      .customRangeRow {
        display: none;
        flex-direction: column;
        align-items: stretch;
        gap: 10px;
        width: 100%;
        box-sizing: border-box;
        margin-top: 2px;
        padding: 12px 14px;
        border-radius: 14px;
        border: 1px solid rgba(255, 255, 255, 0.28);
        background: rgba(8, 14, 28, 0.72);
        box-shadow: 0 4px 18px rgba(0, 0, 0, 0.35);
        font-size: 13px;
        font-weight: 600;
        color: #e8f0ff;
      }
      .customRangeRow.visible { display: flex; }
      .customRangeRow.customFrozen {
        pointer-events: none;
        user-select: none;
        -webkit-user-select: none;
      }
      .customPanelTitle {
        font-size: 12px;
        font-weight: 800;
        letter-spacing: 0.04em;
        text-transform: uppercase;
        color: #9ec8ff;
        margin: 0;
        line-height: 1.3;
      }
      .customPanelHint {
        margin: 0;
        font-size: 11px;
        font-weight: 600;
        line-height: 1.45;
        color: rgba(220, 232, 255, 0.78);
        opacity: 1;
      }
      .customSpeedGrid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 12px 14px;
        width: 100%;
        min-width: 0;
      }
      @media (max-width: 360px) {
        .customSpeedGrid { grid-template-columns: 1fr; }
      }
      .customRangeRow .field {
        display: flex;
        flex-direction: column;
        gap: 6px;
        min-width: 0;
      }
      .customRangeRow .field span {
        font-size: 12px;
        font-weight: 800;
        color: #c5d8ff;
        opacity: 1;
      }
      .customRangeRow .inlineInput {
        width: 100%;
        max-width: 100%;
        box-sizing: border-box;
        min-height: 44px;
        padding: 10px 12px;
        font-size: 16px;
        font-weight: 700;
        border-radius: 12px;
        border: 1px solid rgba(255, 255, 255, 0.42);
        background: rgba(255, 255, 255, 0.12);
      }
      .customRangeRow .inlineInput:focus {
        outline: none;
        border-color: rgba(120, 200, 255, 0.85);
        background: rgba(255, 255, 255, 0.16);
      }
      .customRangeRow .inlineInput:disabled {
        opacity: 0.5;
        cursor: not-allowed;
        border-color: rgba(255, 255, 255, 0.2);
      }
      .customApplyRow {
        display: flex;
        justify-content: flex-end;
        margin-top: 2px;
      }
      button.btnCustomApply {
        border: 1px solid rgba(140, 210, 255, 0.55);
        background: rgba(60, 120, 200, 0.35);
        color: #e8f4ff;
        border-radius: 10px;
        padding: 10px 18px;
        font-size: 13px;
        font-weight: 800;
        min-height: 44px;
        cursor: pointer;
      }
      button.btnCustomApply:hover:not(:disabled) {
        background: rgba(80, 150, 230, 0.45);
        border-color: rgba(180, 230, 255, 0.75);
      }
      button.btnCustomApply:active:not(:disabled) {
        background: rgba(50, 140, 95, 0.55);
        border-color: rgba(140, 255, 190, 0.85);
        transform: scale(0.97);
      }
      button.btnCustomApply.btnCustomApply--ok {
        background: rgba(35, 150, 85, 0.85) !important;
        border-color: rgba(130, 255, 180, 0.95) !important;
        box-shadow: 0 0 0 2px rgba(80, 220, 140, 0.45);
        color: #f4fff8;
      }
      button.btnCustomApply:disabled {
        opacity: 0.4;
        cursor: not-allowed;
      }
      .dynoHelpCard {
        display: none;
        margin-top: 12px;
        padding: 12px 14px;
        border-radius: 14px;
        border: 1px solid rgba(120, 200, 255, 0.35);
        background: rgba(40, 90, 140, 0.18);
        font-size: 12px;
        line-height: 1.45;
        color: #e8f2ff;
        max-width: 720px;
      }
      .dynoHelpCard.visible {
        display: block;
        scroll-margin-bottom: min(340px, 48vh);
      }
      .dynoHelpCard h3 {
        margin: 0 0 8px;
        font-size: 13px;
        font-weight: 800;
        color: #9fd4ff;
      }
      .dynoHelpCard ul { margin: 0; padding-left: 1.15rem; }
      .dynoHelpCard li { margin: 4px 0; }
      .dynoHelpCard .warn { margin-top: 10px; font-size: 11px; opacity: 0.9; color: #ffb8b8; }
      .abortBtn {
        position: fixed;
        right: 14px;
        bottom: 14px;
        z-index: 10000;
        border: 2px solid rgba(255,110,110,0.9);
        background: rgba(180,0,0,0.88);
        color: #fff;
        border-radius: 14px;
        padding: 14px 20px;
        font-weight: 900;
        font-size: 16px;
      }
      .abortBtn:disabled {
        opacity: 0.35;
        cursor: not-allowed;
        pointer-events: none;
      }
      .flashOverlay {
        position: fixed;
        inset: 0;
        pointer-events: none;
        opacity: 0;
        transition: opacity 160ms ease-out;
        z-index: 9999;
      }
      .transitionOverlay {
        position: fixed;
        inset: 0;
        background: rgba(0,0,0,0.5);
        pointer-events: none;
        opacity: 0;
        transition: opacity 200ms ease-out;
        z-index: 10000;
        display: flex;
        align-items: center;
        justify-content: center;
        color: white;
        font-size: 18px;
        font-weight: bold;
      }
      .rotateHint {
        position: fixed;
        inset: 0;
        display: none;
        align-items: center;
        justify-content: center;
        text-align: center;
        background: rgba(0,0,0,0.88);
        color: #fff;
        z-index: 10001;
        font-size: 18px;
        font-weight: 800;
        padding: 20px;
      }
      /* Full-screen start/finish cue when real start/stop conditions fire (not when only arming). */
      .runCueOverlay {
        position: fixed;
        inset: 0;
        z-index: 10005;
        display: none;
        align-items: center;
        justify-content: center;
        text-align: center;
        padding: max(16px, env(safe-area-inset-top)) max(16px, env(safe-area-inset-right))
          max(16px, env(safe-area-inset-bottom)) max(16px, env(safe-area-inset-left));
        box-sizing: border-box;
        background: rgba(0, 0, 0, 0.82);
        pointer-events: auto;
        cursor: pointer;
        -webkit-tap-highlight-color: transparent;
      }
      .runCueOverlay.visible {
        display: flex;
      }
      .runCueOverlayText {
        font-size: clamp(56px, 18vw, 140px);
        font-weight: 900;
        letter-spacing: 0.06em;
        line-height: 1;
        text-transform: uppercase;
        color: #fff;
        text-shadow: 0 0 40px rgba(0, 255, 160, 0.45), 0 4px 24px rgba(0, 0, 0, 0.9);
      }
      .runCueOverlay.runCueOverlay--go .runCueOverlayText {
        color: #00ffaa;
        text-shadow: 0 0 48px rgba(0, 255, 180, 0.55), 0 4px 24px rgba(0, 0, 0, 0.9);
      }
      .runCueOverlay.runCueOverlay--go {
        background: green;
      }
      .runCueOverlay.runCueOverlay--finish .runCueOverlayText {
        color: #ffd84d;
        text-shadow: 0 0 48px rgba(255, 210, 80, 0.5), 0 4px 24px rgba(0, 0, 0, 0.9);
      }
      .runCueOverlay.runCueOverlay--finish {
        background: yellow;
      }
      .runCueOverlay.runCueOverlay--abort {
        background: red;
      }
      .runCueOverlay.runCueOverlay--result .runCueOverlayText {
        color: #ffffff;
        text-shadow: 0 0 48px rgba(255, 255, 255, 0.6), 0 4px 24px rgba(0, 0, 0, 0.9);
      }
      .runCueOverlayHint {
        margin-top: 18px;
        font-size: 13px;
        font-weight: 700;
        color: rgba(230, 240, 255, 0.72);
        letter-spacing: 0.04em;
      }
      .dtModal {
        position: fixed;
        inset: 0;
        z-index: 10002;
        display: none;
        align-items: center;
        justify-content: center;
        padding: 20px;
        overflow-y: auto;
        -webkit-overflow-scrolling: touch;
      }
      .dtModalBackdrop {
        position: absolute;
        inset: 0;
        background: rgba(0, 0, 0, 0.72);
      }
      .dtModalBox {
        position: relative;
        max-width: 420px;
        width: 100%;
        border-radius: 14px;
        border: 1px solid rgba(255, 255, 255, 0.35);
        background: rgba(18, 22, 32, 0.96);
        padding: 18px 18px 14px;
        box-shadow: 0 12px 40px rgba(0, 0, 0, 0.55);
      }
      .dtModalTitle {
        font-size: 17px;
        font-weight: 900;
        color: #fff;
        margin-bottom: 10px;
        letter-spacing: 0.02em;
      }
      .dtModalMsg {
        font-size: 14px;
        line-height: 1.45;
        color: #e8f0ff;
        margin-bottom: 16px;
      }
      .dtModalOk {
        width: 100%;
        font-weight: 900;
      }
      @media (orientation: landscape) and (max-height: 560px) and (hover: none) and (pointer: coarse) {
        .dtModal {
          align-items: flex-start;
          padding: 10px max(12px, env(safe-area-inset-right)) 10px max(12px, env(safe-area-inset-left));
        }
        .dtModalBox {
          margin-top: max(4px, env(safe-area-inset-top));
          margin-bottom: max(4px, env(safe-area-inset-bottom));
          max-height: calc(100dvh - 20px);
          display: flex;
          flex-direction: column;
        }
        .dtModalMsg {
          overflow-y: auto;
          -webkit-overflow-scrolling: touch;
          max-height: calc(100dvh - 210px);
          padding-right: 3px;
          margin-bottom: 12px;
        }
        .dtModalOk {
          position: sticky;
          bottom: 0;
          z-index: 1;
        }
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
      .lockedHint {
        position: fixed;
        right: 14px;
        left: auto;
        bottom: 72px;
        max-width: min(380px, calc(100vw - 100px));
        text-align: right;
        z-index: 9999;
        border: 1px solid rgba(255,255,255,0.35);
        background: rgba(0,0,0,0.72);
        color: #fff;
        border-radius: 10px;
        padding: 10px 12px;
        font-size: 12px;
        font-weight: 700;
        display: none;
      }
    </style>
  </head>
  <body>
    <header>
      <div class="headerLeft">
        <img src="/logo.png" class="brandLogo" width="260" height="92" alt="DYNOTRACK X — Track. Analyze. Improve."/>
      </div>
      <div class="headerCenter">
        <div id="wsState" class="wsBadge ws-connecting">OBDII: connecting…</div>
      </div>
      <div class="headerRight">
        <div class="batteryBlock" id="batteryBlock" title="Remaining charge for the device. Green = full, yellow = charge soon, red = low. With external power to the unit, level is not shown.">
          <div class="batteryBlockTitle">Battery</div>
          <div class="batteryKeyLegend" aria-label="Battery color key">
            <span><span class="bkDot bkDot--g" aria-hidden="true"></span>full</span>
            <span><span class="bkDot bkDot--y" aria-hidden="true"></span>charge soon</span>
            <span><span class="bkDot bkDot--r" aria-hidden="true"></span>low</span>
          </div>
          <div id="batteryWidget" class="batteryWidget state-unknown">
            <div class="batteryIcon"><div id="batteryFill" class="batteryFill"></div></div>
            <div class="batteryInfoCol">
              <div class="batRow"><span class="batLbl">Level</span><span id="batteryPctVal" class="batVal">—</span></div>
              <div class="batRow"><span class="batLbl">Status</span><span id="batteryStatusVal" class="batVal">—</span></div>
            </div>
          </div>
        </div>
      </div>
    </header>

    <div class="wrap">
      <div class="printHeader">
        <h2>DynoTrack X - Dyno Report</h2>
        <div id="printMeta">Generated: -</div>
      </div>
      <div class="banner" id="setupBanner">
        <strong id="setupTitle">Complete setup to start measurement</strong>
        <span id="setupMsg"></span>
        <a id="setupLink" href="/settings">SETUP</a>
      </div>
      <div class="banner" id="calBanner">
        <strong>Calibration invalid</strong>
        <span id="calMsg">Repeat coast-down calibration procedure.</span>
        <div class="calButtons">
          <button id="btnCalHelp">HELP</button>
          <button id="btnCalibrate">CALIBRATE</button>
        </div>
      </div>

      <div class="subNavDashBack" id="subNavDashBack">
        <button type="button" id="btnBackHome" class="screenBtn">← DASHBOARD</button>
      </div>

      <div class="screenBlock active" data-screen="home">
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

      <div class="gaugeRow" id="analogGaugesAnchor" aria-label="Analog gauges">
        <div class="gaugeCard"><canvas id="gaugeSpeed" class="gaugeCanvas" width="340" height="180"></canvas></div>
        <div class="gaugeCard"><canvas id="gaugeRpm" class="gaugeCanvas" width="340" height="180"></canvas></div>
      </div>
      <div class="gaugeRow">
        <div class="gaugeCard"><canvas id="gaugePower" class="gaugeCanvas" width="340" height="180"></canvas></div>
        <div class="gaugeCard"><canvas id="gaugeTorque" class="gaugeCanvas" width="340" height="180"></canvas></div>
      </div>
      <div class="measurementGrid">
        <div class="measureCard">
          <div class="measureLabel">POWER @ WHEELS</div>
          <div class="measureValue"><span id="msPowerWheel">0</span> <span id="msPowerWheelUnit" class="unit">hp</span></div>
        </div>
        <div class="measureCard">
          <div class="measureLabel">G-FORCE (long)</div>
          <div class="measureValue"><span id="msG">0.00</span> <span class="unit">g</span></div>
        </div>
        <div class="measureCard">
          <div class="measureLabel">RUN TIME</div>
          <div class="measureValue"><span id="msRunTime">0.00</span> <span class="unit">s</span></div>
        </div>
        <div class="measureCard">
          <div class="measureLabel">DISTANCE</div>
          <div class="measureValue"><span id="msRunDistance">0.0</span> <span class="unit">m</span></div>
        </div>
      </div>
      <div id="gpsConfidenceBadge" class="gpsConfidenceBadge health-hot">
        GNSS confidence: 0% (waiting lock) - place device under windshield with clear sky view.
      </div>

      <div id="homeStatusRow1" class="status measurementLiveStatus">
        <div class="pill">Dummy push: ~8 Hz</div>
        <div class="pill" id="tInfo">t_ms: 0</div>
        <div class="pill" id="gpsLockInfo">GPS: searching...</div>
        <div class="pill" id="gpsConstellationInfo">GNSS target: GPS+GLONASS+Galileo+BeiDou (up to 4 constellations at once, many satellites). SBAS/QZSS assist only.</div>
        <div class="pill" id="apClientsInfo">AP clients: 0</div>
        <div class="pill" id="lossInfo">Loss total: 0.0 hp</div>
        <div class="pill" id="corrInfo">Corr: DIN x1.000</div>
        <div class="pill" id="anomalyInfo">Anomaly: none</div>
        <div class="pill" id="selfCalInfo">SelfCal: ON (0%)</div>
        <div class="pill" id="iatHealth">IAT: normal</div>
        <div class="pill" id="oilHealth">Oil: normal</div>
      </div>

      <div id="homeStatusRow2" class="status measurementLiveStatus">
        <div class="pill" id="noiseInfo">Noise: 0%</div>
        <div class="pill" id="driftInfo">Drift: 0%</div>
        <div class="pill" id="resInfo">Resolution: 100%</div>
        <div class="pill" id="qualityInfo">Quality: -</div>
        <div class="pill" id="fusionInfo">Fusion: OBD 100% / GPS 0%</div>
        <div class="pill" id="slopeInfo">Auto slope: 0.0%</div>
        <div class="pill" id="windInfo">Wind est: 0.0 km/h</div>
        <div class="pill" id="gearInfo">Gear: -</div>
        <div class="pill" id="slipInfo">Slip: 0.0%</div>
        <div class="pill" id="lastLiveInfo">Last live update: -</div>
      </div>
      <div id="homeStatusRow3" class="status measurementLiveStatus">
        <div class="pill" id="coastCalInfo">CoastCal: not valid (repeat)</div>
        <div class="pill" id="autoRunInfo">AutoRun: idle</div>
        <div class="pill" id="autoRunReasonInfo">Reason: -</div>
      </div>

      <div id="dynoModeHelp" class="dynoHelpCard">
        <h3>Dyno mode — how to drive (auto start / stop)</h3>
        <ul>
          <li><b>Gear &amp; speed:</b> Choose a gear with enough headroom to rev cleanly (e.g. 2nd or 3rd). Hold a <b>steady road speed</b> that matches that gear — the run is tracked by <b>RPM and throttle</b>, not by a km/h window.</li>
          <li><b>Arm:</b> Select <b>Dyno mode</b> below — it arms automatically on the dashboard. After <b>ABORT</b>, slow briefly so speed drops, then accelerate again to re-arm (threshold in <b>Settings</b> → Auto-arm GPS).</li>
          <li><b>Auto start:</b> The run begins when <b>RPM is at least ~1800</b> and <b>throttle is above ~40%</b>. Apply throttle smoothly so both conditions are met together.</li>
          <li><b>During the pull:</b> Keep <b>wide-open throttle (WOT)</b> through the rev range. Avoid lifting early — the app expects a full pull unless you <b>ABORT</b>.</li>
          <li><b>Auto stop:</b> The run ends when <b>RPM goes above ~6400</b>, or after the pull has peaked and RPM <b>falls by about 350 RPM</b> from that peak (typical end of pull / shift).</li>
          <li><b>Cancel anytime:</b> <b>ABORT</b> stops the run and unlocks the screen.</li>
        </ul>
        <div class="warn">Only use full-throttle pulls where legally and safely allowed. You are responsible for vehicle control and road conditions.</div>
      </div>

      <div class="liveBottomBar">
        <div class="liveBottomBarInner">
          <div class="liveBottomBarMenuRow">
          <div class="measurementMenuBlock" id="measurementMenuBlock">
            <span class="measurementMenuTitle">MEASUREMENT MENU</span>
            <select id="measurementMode" class="modeSelect modeRequired" title="Choose a mode — list closes after selection; current mode is shown below">
              <option value="" selected>Select mode…</option>
              <optgroup label="Drag (speed / distance)">
                <option value="drag_0_100">0-100 km/h</option>
                <option value="drag_0_200">0-200 km/h</option>
                <option value="drag_custom">Custom (speed range)</option>
              </optgroup>
              <optgroup label="Drag strip (fixed distance)">
                <option value="drag_201m">1/8 mile (~201 m)</option>
                <option value="drag_402m">1/4 mile (402 m)</option>
                <option value="drag_804m">1/2 mile (~805 m)</option>
                <option value="drag_custom_dist">Custom distance (metres)</option>
              </optgroup>
              <optgroup label="Rolling acceleration (mid-range)">
                <option value="mid_60_100">60-100 km/h</option>
                <option value="mid_80_120">80-120 km/h</option>
                <option value="mid_100_200">100-200 km/h</option>
                <option value="mid_custom">Custom (speed range)</option>
              </optgroup>
              <optgroup label="Other">
                <option value="braking_100_0">100-0 km/h braking</option>
                <option value="braking_custom">Custom braking (speed range)</option>
                <option value="dyno_pull">Dyno mode</option>
              </optgroup>
              <optgroup label="Session">
                <option value="__track_nav__">TRACK (laps)</option>
              </optgroup>
            </select>
            <span id="measurementModeSummary" class="measurementModeSummary" aria-live="polite"></span>
            <div id="measurementCloseRow" class="measurementCloseRow" aria-hidden="true">
              <button type="button" id="btnMeasurementClose" class="screenBtn measurementCloseBtn">CLOSE</button>
              <span class="measurementCloseHint">Clear custom setup and choose another mode</span>
            </div>
            <div id="customSpeedPanel" class="customRangeRow" aria-label="Custom speed range">
              <p class="customPanelTitle">Custom speed (km/h)</p>
              <div class="customSpeedGrid">
                <div class="field">
                  <span>From (km/h)</span>
                  <input id="customStart" class="inlineInput" type="number" min="0" max="320" step="1" value="0" inputmode="numeric" title="Start speed km/h"/>
                </div>
                <div class="field">
                  <span>To (km/h)</span>
                  <input id="customEnd" class="inlineInput" type="number" min="0" max="320" step="1" value="100" inputmode="numeric" title="End speed km/h"/>
                </div>
              </div>
              <div class="customApplyRow">
                <button type="button" id="btnCustomSpeedApply" class="btnCustomApply">Apply</button>
              </div>
              <p id="customRangeHint" class="customPanelHint">Acceleration: From must be lower than To.</p>
            </div>
            <div id="customDragDistRow" class="customRangeRow" aria-label="Custom drag distance">
              <p class="customPanelTitle">Your distance (metres)</p>
              <div class="field">
                <span>Distance (m)</span>
                <input id="customDragDistM" class="inlineInput" type="number" min="5" max="5000" step="1" value=""
                  inputmode="numeric" autocomplete="off" placeholder="e.g. 350"
                  title="Run ends when integrated distance reaches this value (5–5000 m). Use menu presets for ⅛ / ¼ / ½ mile."/>
              </div>
              <div class="customApplyRow">
                <button type="button" id="btnCustomDistApply" class="btnCustomApply">Apply</button>
              </div>
              <p class="customPanelHint">Type any length in <b>metres</b> (5-5000), then <b>Enter</b> or <b>Apply</b>. For 1/8, 1/4 or 1/2 mile use the menu presets above.</p>
            </div>
          </div>
          </div>
          <div class="liveActionsRow">
            <button type="button" id="btnStartRun" class="btn btnGreen btnStartRun" aria-pressed="false">START RUN</button>
            <button type="button" id="btnResultsScreen" class="screenBtn">RESULTS</button>
            <button type="button" id="btnExportCsv" class="btn">EXPORT DATA</button>
            <button type="button" id="btnPrintReport" class="btn">PRINT REPORT</button>
          </div>
        </div>
      </div>

      <div class="settingsBottomLeft"><a class="settingsLink" href="/settings">SETTINGS</a></div>

      <div id="trackModePanel" class="trackModePanel">
        <div class="reportCard">
          <div class="trackModePanelHead">
            <div class="label">TRACK MODE</div>
            <button type="button" id="btnTrackPanelClose" class="screenBtn">CLOSE</button>
          </div>
          <div class="status">
            <div class="pill" id="trackSessionInfo">Session: idle</div>
            <div class="pill" id="trackLapInfo">Lap: -</div>
            <div class="pill" id="trackBestInfo">Best: -</div>
          </div>
          <div class="controls">
            <button id="btnTrackStart" class="btn">RESET SESSION</button>
            <button id="btnTrackStop" class="btn" disabled>STOP SESSION</button>
            <button id="btnTrackExportCsv" class="btn">EXPORT TRACK CSV</button>
          </div>
          <div id="trackSmartHint" class="msg trackSmartHint">Open Track from the menu — session starts automatically and scrolls here.</div>
        </div>
        <div class="runsCard">
          <div class="label">LAP HISTORY</div>
          <div id="trackLapList" class="runsList"></div>
        </div>
      </div>

      <div class="controls">
        <button id="btnTogglePowerUnit" class="unitPill" style="display:none;">Power Unit: HP</button>
      </div>
      </div>

      <div id="dynoGraphCard" class="chartCard screenBlock" data-screen="results">
        <div class="label">Dyno graph (1500 rpm → redline · HP / Nm / loss)</div>
        <canvas id="dynoCanvas" width="900" height="260"></canvas>
      </div>

      <div id="dynoSummaryCard" class="reportCard screenBlock" data-screen="results">
        <div class="label">Dyno Report Summary</div>
        <table class="reportTable">
          <tr><td>Peak Power</td><td id="peakPowerInfo">-</td></tr>
          <tr><td>Peak Torque</td><td id="peakTorqueInfo">-</td></tr>
          <tr><td>WHP vs Engine HP @ peak corr.</td><td id="wheelVsEngineInfo">-</td></tr>
          <tr><td>Loss @ peak corr. (total / aero / roll / slope)</td><td id="dynoLossAtPeakInfo">-</td></tr>
          <tr><td>Drivetrain (gearbox %)</td><td id="driveLossInfo">-</td></tr>
          <tr><td>Correction</td><td id="corrSummaryInfo">-</td></tr>
          <tr><td>Max slip / fuel (avg · max L/h)</td><td id="dynoSlipFuelInfo">-</td></tr>
          <tr><td>Ambient (T/P/H)</td><td id="ambientInfo">-</td></tr>
        </table>
        <div class="msg">Crank power is estimated based on drivetrain and road-load losses.</div>
      </div>
      <div class="reportCard screenBlock" data-screen="results">
        <div class="label">Measurement Result</div>
        <table id="measurementResultTable" class="reportTable">
          <tr><td>Mode</td><td id="mrMode">-</td></tr>
          <tr><td>Status</td><td id="mrStatus">-</td></tr>
          <tr id="mrTrackLapsRow" style="display:none"><td>Laps recorded</td><td id="mrTrackLaps">-</td></tr>
          <tr id="mrTrackBestRow" style="display:none"><td>Best lap</td><td id="mrTrackBest">-</td></tr>
          <tr id="mrTrackAvgRow" style="display:none"><td>Average lap</td><td id="mrTrackAvg">-</td></tr>
          <tr id="mrTrackLastRow" style="display:none"><td>Last lap</td><td id="mrTrackLast">-</td></tr>
          <tr><td>Duration</td><td id="mrTime">-</td></tr>
          <tr id="mrDistanceRow"><td>Distance</td><td id="mrDistance">-</td></tr>
          <tr><td>Speed window (start → end)</td><td id="mrSpeedWindow">-</td></tr>
          <tr><td>Avg / max / min speed</td><td id="mrSpeedStats">-</td></tr>
          <tr><td>Peak power (corr.) @ RPM</td><td id="mrPeakPower">-</td></tr>
          <tr><td>Peak torque @ RPM</td><td id="mrPeakTorque">-</td></tr>
          <tr><td>Max RPM (during run)</td><td id="mrPeakRpm">-</td></tr>
          <tr><td>RPM (start → end)</td><td id="mrRpmWindow">-</td></tr>
          <tr><td>Max throttle</td><td id="mrMaxThrottle">-</td></tr>
          <tr><td>Power @ peak corr. (WHP / crank / indicated)</td><td id="mrPowerSplit">-</td></tr>
          <tr><td>Road-load losses @ peak corr.</td><td id="mrLossBreakdown">-</td></tr>
          <tr><td>Max tire slip</td><td id="mrMaxSlip">-</td></tr>
          <tr><td>Fuel rate (avg / max)</td><td id="mrFuel">-</td></tr>
          <tr><td>Engine oil (start → end)</td><td id="mrEngineOil">-</td></tr>
          <tr><td>Air density</td><td id="mrAirDensity">-</td></tr>
          <tr><td>Correction (std / K)</td><td id="mrCorrection">-</td></tr>
          <tr><td>Ambient (IAT / P / RH)</td><td id="mrAmbient">-</td></tr>
          <tr><td>GPS</td><td id="mrGps">-</td></tr>
          <tr><td>Drive / gearbox loss</td><td id="mrVehicle">-</td></tr>
        </table>
      </div>

    </div>
    <button id="btnAbort" class="abortBtn" disabled title="Available when a run is armed or active">ABORT</button>
    <div id="lockedHint" class="lockedHint">UI locked — use ABORT to cancel or wait for run to finish / timeout.</div>
    <div id="flashOverlay" class="flashOverlay"></div>
    <div id="transitionOverlay" class="transitionOverlay"></div>
    <div id="rotateHint" class="rotateHint">Rotate device to landscape for best live gauges view.</div>
    <div id="runCueOverlay" class="runCueOverlay" role="status" aria-live="assertive" aria-hidden="true">
      <div>
        <div id="runCueOverlayText" class="runCueOverlayText"></div>
        <div id="runCueOverlayHint" class="runCueOverlayHint">Tap to dismiss</div>
      </div>
    </div>
    <div id="dtModal" class="dtModal" style="display:none" role="dialog" aria-modal="true" aria-labelledby="dtModalTitleEl">
      <div class="dtModalBackdrop" id="dtModalBackdrop"></div>
      <div class="dtModalBox">
        <div id="dtModalTitleEl" class="dtModalTitle">DynoTrack X</div>
        <div id="dtModalMsg" class="dtModalMsg"></div>
        <button type="button" id="dtModalOk" class="btn dtModalOk">OK</button>
      </div>
    </div>

    <script>
      const wsUrl = (location.protocol === 'https:' ? 'wss:' : 'ws:') + '//' + location.host + '/ws';
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
      const elSelfCalInfo = document.getElementById('selfCalInfo');
      const elNoiseInfo = document.getElementById('noiseInfo');
      const elDriftInfo = document.getElementById('driftInfo');
      const elResInfo = document.getElementById('resInfo');
      const elQualityInfo = document.getElementById('qualityInfo');
      const elFusionInfo = document.getElementById('fusionInfo');
      const elSlopeInfo = document.getElementById('slopeInfo');
      const elWindInfo = document.getElementById('windInfo');
      const elGearInfo = document.getElementById('gearInfo');
      const elSlipInfo = document.getElementById('slipInfo');
      const elAutoRunInfo = document.getElementById('autoRunInfo');
      const elAutoRunReasonInfo = document.getElementById('autoRunReasonInfo');
      const elCoastCalInfo = document.getElementById('coastCalInfo');
      const elIatHealth = document.getElementById('iatHealth');
      const elOilHealth = document.getElementById('oilHealth');
      const iatCard = document.getElementById('iatCard');
      const oilCard = document.getElementById('oilCard');
      const elTInfo = document.getElementById('tInfo');
      const elGpsLockInfo = document.getElementById('gpsLockInfo');
      const elGpsConstellationInfo = document.getElementById('gpsConstellationInfo');
      const elGpsConfidenceBadge = document.getElementById('gpsConfidenceBadge');
      const elApClientsInfo = document.getElementById('apClientsInfo');
      const elLastLiveInfo = document.getElementById('lastLiveInfo');
      const batteryWidget = document.getElementById('batteryWidget');
      const batteryFill = document.getElementById('batteryFill');
      const batteryPctVal = document.getElementById('batteryPctVal');
      const batteryStatusVal = document.getElementById('batteryStatusVal');
      const btnAbort = document.getElementById('btnAbort');
      const btnStartRun = document.getElementById('btnStartRun');
      const subNavDashBack = document.getElementById('subNavDashBack');
      const btnBackHome = document.getElementById('btnBackHome');
      const btnResultsScreen = document.getElementById('btnResultsScreen');
      const mainWrap = document.querySelector('.wrap');
      const measurementModeEl = document.getElementById('measurementMode');
      const btnExportCsv = document.getElementById('btnExportCsv');
      const btnPrintReport = document.getElementById('btnPrintReport');
      const btnTogglePowerUnit = document.getElementById('btnTogglePowerUnit');
      const printMeta = document.getElementById('printMeta');
      const customStartEl = document.getElementById('customStart');
      const customEndEl = document.getElementById('customEnd');
      const customSpeedPanel = document.getElementById('customSpeedPanel');
      const customDragDistRow = document.getElementById('customDragDistRow');
      const customDragDistMEl = document.getElementById('customDragDistM');
      const lockedHint = document.getElementById('lockedHint');
      const rotateHint = document.getElementById('rotateHint');
      const peakPowerInfo = document.getElementById('peakPowerInfo');
      const peakTorqueInfo = document.getElementById('peakTorqueInfo');
      const wheelVsEngineInfo = document.getElementById('wheelVsEngineInfo');
      const driveLossInfo = document.getElementById('driveLossInfo');
      const corrSummaryInfo = document.getElementById('corrSummaryInfo');
      const dynoLossAtPeakInfo = document.getElementById('dynoLossAtPeakInfo');
      const dynoSlipFuelInfo = document.getElementById('dynoSlipFuelInfo');
      const ambientInfo = document.getElementById('ambientInfo');
      const mrMode = document.getElementById('mrMode');
      const mrTime = document.getElementById('mrTime');
      const mrDistance = document.getElementById('mrDistance');
      const mrDistanceRow = document.getElementById('mrDistanceRow');
      const mrSpeedWindow = document.getElementById('mrSpeedWindow');
      const mrSpeedStats = document.getElementById('mrSpeedStats');
      const mrPeakPower = document.getElementById('mrPeakPower');
      const mrPeakTorque = document.getElementById('mrPeakTorque');
      const mrPeakRpm = document.getElementById('mrPeakRpm');
      const mrRpmWindow = document.getElementById('mrRpmWindow');
      const mrMaxThrottle = document.getElementById('mrMaxThrottle');
      const mrAmbient = document.getElementById('mrAmbient');
      const mrGps = document.getElementById('mrGps');
      const mrVehicle = document.getElementById('mrVehicle');
      const mrPowerSplit = document.getElementById('mrPowerSplit');
      const mrLossBreakdown = document.getElementById('mrLossBreakdown');
      const mrMaxSlip = document.getElementById('mrMaxSlip');
      const mrFuel = document.getElementById('mrFuel');
      const mrEngineOil = document.getElementById('mrEngineOil');
      const mrAirDensity = document.getElementById('mrAirDensity');
      const mrCorrection = document.getElementById('mrCorrection');
      const mrStatus = document.getElementById('mrStatus');
      const mrTrackLapsRow = document.getElementById('mrTrackLapsRow');
      const mrTrackBestRow = document.getElementById('mrTrackBestRow');
      const mrTrackAvgRow = document.getElementById('mrTrackAvgRow');
      const mrTrackLastRow = document.getElementById('mrTrackLastRow');
      const mrTrackLaps = document.getElementById('mrTrackLaps');
      const mrTrackBest = document.getElementById('mrTrackBest');
      const mrTrackAvg = document.getElementById('mrTrackAvg');
      const mrTrackLast = document.getElementById('mrTrackLast');
      const dynoGraphCard = document.getElementById('dynoGraphCard');
      const dynoSummaryCard = document.getElementById('dynoSummaryCard');
      const canvas = document.getElementById('dynoCanvas');
      const ctx = canvas.getContext('2d');
      const msPowerWheel = document.getElementById('msPowerWheel');
      const msPowerWheelUnit = document.getElementById('msPowerWheelUnit');
      const msG = document.getElementById('msG');
      const msRunTime = document.getElementById('msRunTime');
      const msRunDistance = document.getElementById('msRunDistance');
      const gaugeSpeed = document.getElementById('gaugeSpeed');
      const gaugeRpm = document.getElementById('gaugeRpm');
      const gaugePower = document.getElementById('gaugePower');
      const gaugeTorque = document.getElementById('gaugeTorque');
      const flashOverlay = document.getElementById('flashOverlay');
      const transitionOverlay = document.getElementById('transitionOverlay');
      const dtModal = document.getElementById('dtModal');
      const dtModalMsg = document.getElementById('dtModalMsg');
      const dtModalOk = document.getElementById('dtModalOk');
      const dtModalBackdrop = document.getElementById('dtModalBackdrop');
      const trackSessionInfo = document.getElementById('trackSessionInfo');
      const trackLapInfo = document.getElementById('trackLapInfo');
      const trackBestInfo = document.getElementById('trackBestInfo');
      const trackLapList = document.getElementById('trackLapList');
      const btnTrackStart = document.getElementById('btnTrackStart');
      const btnTrackStop = document.getElementById('btnTrackStop');
      const btnTrackExportCsv = document.getElementById('btnTrackExportCsv');
      const trackModePanel = document.getElementById('trackModePanel');
      const btnTrackPanelClose = document.getElementById('btnTrackPanelClose');
      const measurementMenuBlockEl = document.getElementById('measurementMenuBlock');
      const setupBanner = document.getElementById('setupBanner');
      const setupTitle = document.getElementById('setupTitle');
      const setupMsg = document.getElementById('setupMsg');
      const setupLink = document.getElementById('setupLink');
      const calBanner = document.getElementById('calBanner');
      const calMsg = document.getElementById('calMsg');
      const btnCalHelp = document.getElementById('btnCalHelp');
      const btnCalibrate = document.getElementById('btnCalibrate');
      let lastMissingFields = [];
      let coastCalInProgress = false;
      let coastCalBlocksControls = false;
      let runArmed = false;
      let runActive = false;
      let customApplySealActive = false;
      let committedMeasurementMode = '';
      let trackPanelVisible = false;
      let measurementModeBeforeTrack = '';
      let lastMeasurementSummarySnapshotBeforeTrack = null;
      function getMeasurementMode() {
        return committedMeasurementMode || '';
      }
      function resetMeasurementModeSelectDisplay() {
        if (!measurementModeEl) return;
        measurementModeEl.value = '';
      }
      let currentRun = [];
      let savedRuns = [];
      let runCueHideTimer = null;
      const maxRunPoints = 2200;
      let lastDynoDrawAtMs = 0;
      /** Bump to cancel pending refreshDynoResultsCanvas rAF work (avoids heavy drawCurve after leaving Results). */
      let dynoResultsRedrawToken = 0;
      let gaugesLayoutRaf = null;
      let lastReportUpdateAtMs = 0;
      let powerUnit = 'hp';
      let lastLiveMsg = null;
      let lastAccelMps2 = 0;
      let lastUiPaintMs = 0;
      let runReady = false;
      let runDistanceM = 0;
      let runDistancePrevM = 0;
      let lastSampleTms = 0;
      /** Wall-clock start for RUN TIME display (smooth vs device t_ms gaps / reconnect). */
      let runDisplayStartPerf = 0;
      let runStartSpeedKmh = 0;
      let runStartTms = 0;
      let runLastSpeedKmh = 0;
      let runLastSpeedChangeTms = 0;
      let prevRunSample = null;
      let dynoPeakRpm = 0;
      let lastRunElapsedS = 0;
      let lastRunDistanceM = 0;
      let lastWsMsgAt = 0;
      let dtModalOnOk = null;
      let gpsRiskAckForArm = false;
      let runGpsDropDetected = false;
      let runGpsDropNotified = false;
      let lastRunHadGpsDrop = false;
      let runGpsSamples = 0;
      let runGpsConfAcc = 0;
      let runGpsMinConfPct = 100;
      let runGpsMinSats = 99;
      let runGpsMaxHdop = 0;
      let runGpsHadNoLock = false;
      let runStartedWithGpsRisk = false;
      const gpsStatusSnapshot = {
        lock: false,
        sats: 0,
        hdop: 99,
        gnssMode: 'NONE',
        mainReady: false,
        confidencePct: 0
      };
      /** Last GNSS badge tier for one-shot alert sounds (green | yellow | red) — matches #gpsConfidenceBadge logic. */
      let gnssBadgeAlertPrevLevel = null;
      /** Suppress rapid red re-entries when confidence hovers near the 50% threshold (yellow ↔ red flicker). */
      let lastGnssRedCriticalPlayMs = 0;
      const GNSS_RED_ALERT_COOLDOWN_MS = 2800;
      /** Road run: min time between weak-GNSS (lock/sats/HDOP) alerts — same thresholds as isGpsWeakForActiveRun. */
      let lastGpsWeakRoadAlertMs = 0;
      const GPS_WEAK_ROAD_ALERT_COOLDOWN_MS = 5000;
      let lastMainResult = '';
      const lastMeasurementSummary = {
        mode: '-',
        modeKey: '-',
        timeS: 0,
        distanceM: 0,
        startKmh: 0,
        endKmh: 0,
        status: 'idle',
        avgSpeedKmh: NaN,
        maxSpeedKmh: NaN,
        minSpeedKmh: NaN,
        peakHp: NaN,
        peakTorqueNm: NaN,
        peakTorqueAtPeakHp: NaN,
        finalHp: NaN,
        finalTorque: NaN,
        peakRpm: NaN,
        startRpm: NaN,
        endRpm: NaN,
        maxThrottle: NaN,
        ambientTxt: '-',
        gpsTxt: '-',
        vehicleTxt: '-',
        peakHpCorrRpm: NaN,
        peakTorqueRpm: NaN,
        peakHpWheel: NaN,
        peakHpCrank: NaN,
        peakHpIndicated: NaN,
        peakLossTotal: NaN,
        peakLossAero: NaN,
        peakLossRoll: NaN,
        peakLossSlope: NaN,
        maxSlipPct: NaN,
        avgFuelLph: NaN,
        maxFuelLph: NaN,
        engineOilStartC: NaN,
        engineOilEndC: NaN,
        airDensity: NaN,
        corrStd: '',
        corrFactorK: NaN,
        lossGearboxPct: NaN,
        driveType: ''
      };
      let gpsLat = 0;
      let gpsLon = 0;
      let trackSessionActive = false;
      let trackStartLat = null;
      let trackStartLon = null;
      let trackOriginLat = null;
      let trackOriginLon = null;
      let trackMaxDistFromOriginM = 0;
      let trackHasLeftOrigin = false;
      let trackLapStartTms = 0;
      let trackLapInZone = false;
      let trackCurrentLapNo = 0;
      let trackBestLapS = 0;
      let trackCurrentTopSpeed = 0;
      let trackCurrentSpeedSum = 0;
      let trackCurrentSamples = 0;
      let trackCurrentRpmTrend = [];
      let trackCurrentSpeedTrend = [];
      let trackCurrentHpTrend = [];
      let trackLaps = [];
      /** START RUN on Track: gate GPS at button press; lap starts when speed >= Auto-arm km/h (Settings); ends at gate + standstill. */
      let trackStartRunLapArm = false;
      let trackGateLat = null;
      let trackGateLon = null;
      let trackAwaitingSpeedForLap = false;
      let trackRunHasLeftGate = false;
      let redlineRpm = 6500;
      let sessionPeakPower = 0;
      let sessionPeakTorque = 0;
      let suppressAutoArm = false;
      /** Manual START RUN only after a finished run, timeout, or ABORT — not while waiting for auto conditions. */
      let allowManualStartRun = false;
      let activeScreen = 'home';
      let isSwitchingToHome = false;
      let cachedAutoArmKmh = 15;
      /** Mirrors firmware Settings: when false, user must press START RUN after choosing a mode (same start thresholds either way). Track laps unchanged — START RUN still saves the gate. */
      let measurementAutoArmPref = true;
      const TRACK_STANDSTILL_KMH = 4.0;
      const TRACK_MIN_LEAVE_M = 38;
      const TRACK_ZONE_M = 24;
      const TRACK_MIN_LAP_MS = 4200;

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
        if (msPowerWheelUnit) msPowerWheelUnit.textContent = u;
      }

      /* Custom fields: live DOM may hold in-progress typing; measurement logic uses committed values (Enter). */
      let customSpeedCommittedLo = 0;
      let customSpeedCommittedHi = 100;
      let customBrakingCommittedFrom = 80;
      let customBrakingCommittedTo = 20;
      let customDragCommittedM = NaN;

      function commitCustomAccelMidFromInputs() {
        if (!customStartEl || !customEndEl) return;
        let lo = parseInt(customStartEl.value, 10);
        let hi = parseInt(customEndEl.value, 10);
        if (!isFinite(lo)) lo = 0;
        if (!isFinite(hi)) hi = 100;
        if (lo > hi) { const t = lo; lo = hi; hi = t; }
        lo = Math.max(0, Math.min(320, lo));
        hi = Math.max(0, Math.min(320, hi));
        customSpeedCommittedLo = lo;
        customSpeedCommittedHi = hi;
        customStartEl.value = String(lo);
        customEndEl.value = String(hi);
      }

      function commitBrakingFromInputs() {
        if (!customStartEl || !customEndEl) return;
        let from = parseInt(customStartEl.value, 10);
        let to = parseInt(customEndEl.value, 10);
        if (!isFinite(from)) from = 80;
        if (!isFinite(to)) to = 20;
        from = Math.max(0, Math.min(320, from));
        to = Math.max(0, Math.min(320, to));
        customBrakingCommittedFrom = from;
        customBrakingCommittedTo = to;
        customStartEl.value = String(from);
        customEndEl.value = String(to);
      }

      function commitCustomDistFromInputs() {
        if (!customDragDistMEl) return;
        const raw = String(customDragDistMEl.value || '').trim();
        if (!raw) {
          customDragCommittedM = NaN;
          return;
        }
        const v = parseFloat(raw.replace(',', '.'));
        if (!isFinite(v) || v < 5 || v > 5000) {
          customDragCommittedM = NaN;
          return;
        }
        customDragCommittedM = v;
        customDragDistMEl.value = String(Math.round(v));
      }

      function applyCustomFieldsNow(fromBtn) {
        normalizeCustomInputs();
        customApplySealActive = true;
        suppressAutoArm = false;
        allowManualStartRun = false;
        if (!runActive && !runArmed && canAutoArmMeasurement()) {
          if (measurementAutoArmPref) armAutoRunQuiet();
          else allowManualStartRun = true;
        }
        refreshInteractionLock();
        refreshStartRunButton();
        if (fromBtn && fromBtn.classList) {
          fromBtn.classList.add('btnCustomApply--ok');
          window.setTimeout(function () {
            try { fromBtn.classList.remove('btnCustomApply--ok'); } catch (e) {}
          }, 450);
        }
      }

      function onCustomFieldEnter(ev) {
        if (!ev || ev.key !== 'Enter') return;
        ev.preventDefault();
        applyCustomFieldsNow(null);
        if (ev.target && typeof ev.target.blur === 'function') ev.target.blur();
      }

      function parseCustomRange() {
        let lo = customSpeedCommittedLo;
        let hi = customSpeedCommittedHi;
        lo = Math.max(0, Math.min(320, lo));
        hi = Math.max(0, Math.min(320, hi));
        if (lo > hi) { const t = lo; lo = hi; hi = t; }
        return { lo, hi, valid: lo < hi };
      }

      function parseBrakingRange() {
        let from = customBrakingCommittedFrom;
        let to = customBrakingCommittedTo;
        from = Math.max(0, Math.min(320, from));
        to = Math.max(0, Math.min(320, to));
        return { from, to, valid: from > to };
      }

      const DRAG_MILE_M = 1609.344;
      function dragStripPresetM(mode) {
        if (mode === 'drag_201m') return DRAG_MILE_M / 8.0;
        if (mode === 'drag_402m') return 402.0;
        if (mode === 'drag_804m') return DRAG_MILE_M / 2.0;
        return NaN;
      }
      function parseCustomDragDistM() {
        const v = customDragCommittedM;
        if (!isFinite(v) || v < 5 || v > 5000) return { meters: NaN, valid: false };
        return { meters: v, valid: true };
      }

      function normalizeCustomInputs() {
        const mode = getMeasurementMode();
        if (mode === 'braking_custom') commitBrakingFromInputs();
        else if (mode === 'drag_custom' || mode === 'mid_custom') commitCustomAccelMidFromInputs();
        else if (mode === 'drag_custom_dist') commitCustomDistFromInputs();
      }

      function updateCustomRangeVisibility() {
        const m = getMeasurementMode();
        const onHome = activeScreen === 'home';
        const seal = customApplySealActive;
        const lockMeasUi = runArmed || runActive;
        let showSpeedPanel = onHome && (m === 'drag_custom' || m === 'mid_custom' || m === 'braking_custom');
        if (seal && (m === 'drag_custom' || m === 'mid_custom' || m === 'braking_custom')) showSpeedPanel = false;
        if (lockMeasUi && (m === 'drag_custom' || m === 'mid_custom' || m === 'braking_custom')) showSpeedPanel = false;
        if (customSpeedPanel) customSpeedPanel.classList.toggle('visible', showSpeedPanel);
        let showDistRow = onHome && m === 'drag_custom_dist';
        if (seal && m === 'drag_custom_dist') showDistRow = false;
        if (lockMeasUi && m === 'drag_custom_dist') showDistRow = false;
        if (customDragDistRow) customDragDistRow.classList.toggle('visible', showDistRow);
        const hint = document.getElementById('customRangeHint');
        if (hint) {
          hint.textContent = (m === 'braking_custom'
            ? 'Braking: From = start speed, To = target (slower). From must be greater than To.'
            : 'Acceleration / rolling: From must be lower than To (both in km/h).')
            + ' Press Enter or Apply to confirm.';
        }
        const dynoHelp = document.getElementById('dynoModeHelp');
        if (dynoHelp) dynoHelp.classList.toggle('visible', onHome && m === 'dyno_pull');
        const homeScreenEl = document.querySelector('.screenBlock[data-screen="home"]');
        if (homeScreenEl) homeScreenEl.classList.toggle('dynoModeExtraScroll', onHome && m === 'dyno_pull');
      }

      function updateResultsLayout() {
        const aborted = lastMeasurementSummary.status === 'aborted';
        const dynoMode = !aborted && (getMeasurementMode() === 'dyno_pull'
          || lastMeasurementSummary.modeKey === 'dyno_pull');
        if (dynoGraphCard) dynoGraphCard.style.display = dynoMode ? '' : 'none';
        if (dynoSummaryCard) dynoSummaryCard.style.display = dynoMode ? '' : 'none';
      }

      /** Redraw dyno chart on Results after layout has size (also after mode cleared post-run). */
      function refreshDynoResultsCanvas() {
        if (!canvas) return;
        if (lastMeasurementSummary.status === 'aborted') {
          try { drawCurve([]); } catch (e) {}
          return;
        }
        const show = getMeasurementMode() === 'dyno_pull' || lastMeasurementSummary.modeKey === 'dyno_pull';
        if (!show) return;
        const pts = getPointsForExportedRun();
        const myId = ++dynoResultsRedrawToken;
        requestAnimationFrame(() => {
          requestAnimationFrame(() => {
            if (myId !== dynoResultsRedrawToken) return;
            if (activeScreen !== 'results') return;
            try {
              drawCurve(pts.length >= 2 ? pts : []);
            } catch (e) {}
          });
        });
      }

      function modeDisplayLabel(mode) {
        if (mode === 'drag_custom' || mode === 'mid_custom') {
          const r = parseCustomRange();
          return r.valid ? (r.lo + '-' + r.hi + ' km/h') : 'custom speed — enter From/To, then Apply';
        }
        if (mode === 'drag_custom_dist') {
          const d = parseCustomDragDistM();
          return d.valid ? (d.meters.toFixed(0) + ' m (custom)') : 'custom distance — enter metres, then Apply';
        }
        const map = {
          drag_0_100: '0-100',
          drag_0_200: '0-200',
          drag_201m: '1/8 mile',
          drag_402m: '1/4 mile',
          drag_804m: '1/2 mile',
          mid_60_100: '60-100',
          mid_80_120: '80-120',
          mid_100_200: '100-200',
          braking_100_0: '100-0',
          braking_custom: 'custom braking',
          dyno_pull: 'Dyno mode',
          __track_nav__: 'Track (laps)'
        };
        return map[mode] || String(mode).replace(/_/g, '-');
      }

      function refreshModeRequiredStyle() {
        if (measurementModeEl) {
          measurementModeEl.classList.toggle('modeRequired', !getMeasurementMode());
        }
        const sum = document.getElementById('measurementModeSummary');
        if (sum) {
          const m = getMeasurementMode();
          sum.textContent = m ? ('Active mode · ' + modeDisplayLabel(m)) : '';
        }
      }

      function fmtPwrVal(v, u) {
        return isFinite(v) ? (v.toFixed(1) + ' ' + u) : '-';
      }
      function getMainResultString(mode, s) {
        if (mode === 'dyno_pull') {
          const hp = s.peakHp;
          const nm = s.peakTorqueNm;
          if (isFinite(hp) && isFinite(nm)) {
            return hp.toFixed(0) + ' ' + powerUnitLabel() + ' ' + nm.toFixed(0) + 'Nm';
          }
        } else if (mode.startsWith('drag_') || mode === 'mid_custom' || mode.startsWith('mid_')) {
          const time = s.timeS;
          if (isFinite(time)) {
            return time.toFixed(2) + ' sec';
          }
        } else if (mode === 'braking_100_0' || mode === 'braking_custom') {
          const dist = s.distanceM;
          if (isFinite(dist)) {
            return dist.toFixed(0) + 'm';
          }
        } else if (mode === '__track_nav__') {
          // for track, the last lap time
          if (trackLaps.length > 0) {
            const lastLap = trackLaps[trackLaps.length - 1];
            const time = lastLap.time;
            if (isFinite(time)) {
              const min = Math.floor(time / 60);
              const sec = (time % 60).toFixed(2);
              if (min > 0) {
                return min + 'min ' + sec + 'sec';
              } else {
                return sec + ' sec';
              }
            }
          }
        }
        return '';
      }

      function renderMeasurementResult() {
        const isDyno = lastMeasurementSummary.modeKey === 'dyno_pull';
        const isTrack = getMeasurementMode() === '__track_nav__';
        const u = powerUnitLabel();
        const s = lastMeasurementSummary;
        const tro = (row, on) => { if (row) row.style.display = on ? '' : 'none'; };
        if (s.status === 'aborted') {
          const em = '—';
          tro(mrTrackLapsRow, false);
          tro(mrTrackBestRow, false);
          tro(mrTrackAvgRow, false);
          tro(mrTrackLastRow, false);
          if (mrDistanceRow) mrDistanceRow.style.display = '';
          if (mrMode) mrMode.textContent = s.mode || '-';
          if (mrStatus) mrStatus.textContent = 'Aborted';
          if (mrTime) mrTime.textContent = 'Run aborted — no valid measurement data.';
          if (mrDistance) mrDistance.textContent = em;
          if (mrSpeedWindow) mrSpeedWindow.textContent = em;
          if (mrSpeedStats) mrSpeedStats.textContent = em;
          if (mrPeakPower) mrPeakPower.textContent = em;
          if (mrPeakTorque) mrPeakTorque.textContent = em;
          if (mrPeakRpm) mrPeakRpm.textContent = em;
          if (mrRpmWindow) mrRpmWindow.textContent = em;
          if (mrMaxThrottle) mrMaxThrottle.textContent = em;
          if (mrPowerSplit) mrPowerSplit.textContent = em;
          if (mrLossBreakdown) mrLossBreakdown.textContent = em;
          if (mrMaxSlip) mrMaxSlip.textContent = em;
          if (mrFuel) mrFuel.textContent = em;
          if (mrEngineOil) mrEngineOil.textContent = em;
          if (mrAirDensity) mrAirDensity.textContent = em;
          if (mrCorrection) mrCorrection.textContent = em;
          if (mrAmbient) mrAmbient.textContent = em;
          if (mrGps) mrGps.textContent = em;
          if (mrVehicle) mrVehicle.textContent = em;
          return;
        }
        tro(mrTrackLapsRow, isTrack);
        tro(mrTrackBestRow, isTrack);
        tro(mrTrackAvgRow, isTrack);
        tro(mrTrackLastRow, isTrack);
        if (isTrack) {
          const nL = trackLaps.length;
          const lastL = nL ? trackLaps[nL - 1] : null;
          if (mrTrackLaps) mrTrackLaps.textContent = String(nL);
          if (mrTrackBest) mrTrackBest.textContent = trackBestLapS > 0 ? (trackBestLapS.toFixed(2) + ' s') : '—';
          let sumT = 0;
          trackLaps.forEach(l => { sumT += l.time; });
          if (mrTrackAvg) mrTrackAvg.textContent = nL > 0 ? ((sumT / nL).toFixed(2) + ' s') : '—';
          if (mrTrackLast) mrTrackLast.textContent = lastL ? (lastL.time.toFixed(2) + ' s') : '—';
        }
        if (mrDistanceRow) mrDistanceRow.style.display = (isDyno || isTrack) ? 'none' : '';
        if (mrMode) mrMode.textContent = s.mode || '-';
        if (mrStatus) mrStatus.textContent = s.status || '-';
        if (mrTime) {
          if (isTrack) {
            mrTime.textContent = isFinite(s.timeS) ? (s.timeS.toFixed(2) + ' s (best lap)') : '—';
          } else {
            mrTime.textContent = isFinite(s.timeS) ? (s.timeS.toFixed(2) + ' s') : '-';
          }
        }
        if (mrDistance) mrDistance.textContent = isFinite(s.distanceM) ? (s.distanceM.toFixed(1) + ' m') : '-';
        if (mrSpeedWindow) {
          if (isTrack) {
            mrSpeedWindow.textContent = '— (lap session — not a single speed window)';
          } else if (isFinite(s.startKmh) && isFinite(s.endKmh)) {
            mrSpeedWindow.textContent = s.startKmh.toFixed(1) + ' → ' + s.endKmh.toFixed(1) + ' km/h';
          } else mrSpeedWindow.textContent = '-';
        }
        if (mrSpeedStats) {
          if (isTrack) {
            if (isFinite(s.avgSpeedKmh) && isFinite(s.maxSpeedKmh)) {
              mrSpeedStats.textContent = 'Across laps: ' + s.avgSpeedKmh.toFixed(1) + ' avg / ' + s.maxSpeedKmh.toFixed(1) + ' max / '
                + (isFinite(s.minSpeedKmh) ? s.minSpeedKmh.toFixed(1) : '-') + ' min km/h (per-lap top speeds)';
            } else mrSpeedStats.textContent = '—';
          } else if (isFinite(s.avgSpeedKmh) && isFinite(s.maxSpeedKmh)) {
            mrSpeedStats.textContent = s.avgSpeedKmh.toFixed(1) + ' avg / ' + s.maxSpeedKmh.toFixed(1) + ' max / '
              + (isFinite(s.minSpeedKmh) ? s.minSpeedKmh.toFixed(1) : '-') + ' min km/h';
          } else mrSpeedStats.textContent = '-';
        }
        if (mrPeakPower) {
          if (isFinite(s.peakHp) && isFinite(s.peakHpCorrRpm)) {
            mrPeakPower.textContent = s.peakHp.toFixed(1) + ' ' + u + ' (corr.) @ ' + Math.round(s.peakHpCorrRpm) + ' rpm';
          } else if (isFinite(s.peakHp)) {
            mrPeakPower.textContent = s.peakHp.toFixed(1) + ' ' + u + ' (corr.)';
          } else mrPeakPower.textContent = '-';
        }
        if (mrPeakTorque) {
          if (isFinite(s.peakTorqueNm) && isFinite(s.peakTorqueRpm)) {
            mrPeakTorque.textContent = s.peakTorqueNm.toFixed(1) + ' Nm @ ' + Math.round(s.peakTorqueRpm) + ' rpm';
          } else if (isFinite(s.peakTorqueNm)) {
            mrPeakTorque.textContent = s.peakTorqueNm.toFixed(1) + ' Nm';
          } else mrPeakTorque.textContent = '-';
        }
        if (mrPeakRpm) mrPeakRpm.textContent = isFinite(s.peakRpm) ? (String(Math.round(s.peakRpm)) + ' rpm') : '-';
        if (mrRpmWindow) {
          if (isFinite(s.startRpm) && isFinite(s.endRpm)) {
            mrRpmWindow.textContent = Math.round(s.startRpm) + ' → ' + Math.round(s.endRpm) + ' rpm';
          } else mrRpmWindow.textContent = '-';
        }
        if (mrMaxThrottle) mrMaxThrottle.textContent = isFinite(s.maxThrottle) ? (s.maxThrottle.toFixed(0) + ' %') : '-';
        if (mrPowerSplit) {
          const a = fmtPwrVal(s.peakHpWheel, u);
          const b = fmtPwrVal(s.peakHpCrank, u);
          const c = fmtPwrVal(s.peakHpIndicated, u);
          if (a !== '-' || b !== '-' || c !== '-') mrPowerSplit.textContent = a + ' / ' + b + ' / ' + c;
          else mrPowerSplit.textContent = '-';
        }
        if (mrLossBreakdown) {
          const tot = fmtPwrVal(s.peakLossTotal, u);
          const ar = fmtPwrVal(s.peakLossAero, u);
          const ro = fmtPwrVal(s.peakLossRoll, u);
          const sl = fmtPwrVal(s.peakLossSlope, u);
          if (tot !== '-') {
            mrLossBreakdown.textContent = tot + ' total';
            if (ar !== '-' || ro !== '-' || sl !== '-') {
              mrLossBreakdown.textContent += ' (' + ar + ' aero, ' + ro + ' roll, ' + sl + ' slope)';
            }
          } else mrLossBreakdown.textContent = '-';
        }
        if (mrMaxSlip) mrMaxSlip.textContent = isFinite(s.maxSlipPct) ? (s.maxSlipPct.toFixed(1) + ' %') : '-';
        if (mrFuel) {
          if (isFinite(s.avgFuelLph) && isFinite(s.maxFuelLph)) {
            mrFuel.textContent = s.avgFuelLph.toFixed(2) + ' / ' + s.maxFuelLph.toFixed(2) + ' L/h';
          } else mrFuel.textContent = '-';
        }
        if (mrEngineOil) {
          if (isFinite(s.engineOilStartC) && isFinite(s.engineOilEndC)) {
            mrEngineOil.textContent = s.engineOilStartC.toFixed(1) + ' → ' + s.engineOilEndC.toFixed(1) + ' °C';
          } else mrEngineOil.textContent = '-';
        }
        if (mrAirDensity) mrAirDensity.textContent = isFinite(s.airDensity) ? (s.airDensity.toFixed(4) + ' kg/m³') : '-';
        if (mrCorrection) {
          if (s.corrStd && isFinite(s.corrFactorK)) {
            mrCorrection.textContent = String(s.corrStd).toUpperCase() + ' | K=' + s.corrFactorK.toFixed(3);
          } else if (s.corrStd) mrCorrection.textContent = String(s.corrStd).toUpperCase();
          else if (isFinite(s.corrFactorK)) mrCorrection.textContent = 'K=' + s.corrFactorK.toFixed(3);
          else mrCorrection.textContent = '-';
        }
        if (mrAmbient) mrAmbient.textContent = s.ambientTxt || '-';
        if (mrGps) mrGps.textContent = s.gpsTxt || '-';
        if (mrVehicle) {
          const drv = s.driveType ? String(s.driveType).toUpperCase() : '';
          const gb = isFinite(s.lossGearboxPct) ? (s.lossGearboxPct.toFixed(1) + '%') : '';
          if (drv && gb) mrVehicle.textContent = drv + ' | gearbox loss ' + gb;
          else if (drv) mrVehicle.textContent = drv;
          else if (s.vehicleTxt && s.vehicleTxt !== '-') mrVehicle.textContent = s.vehicleTxt;
          else mrVehicle.textContent = '-';
        }
      }

      function applyAbortedMeasurementSummary(mode) {
        seedIdleMeasurementSummary(mode);
        lastMeasurementSummary.modeKey = mode;
        lastMeasurementSummary.mode = modeDisplayLabel(mode);
        lastMeasurementSummary.status = 'aborted';
        lastMeasurementSummary.ambientTxt = '—';
        lastMeasurementSummary.gpsTxt = '—';
        lastMeasurementSummary.vehicleTxt = '—';
        renderMeasurementResult();
        lastMainResult = '';
      }

      function setMeasurementResultFromRun(mode, points, distanceM, statusText, envMsg) {
        if (String(statusText || '').toLowerCase() === 'aborted') {
          if (mode) applyAbortedMeasurementSummary(mode);
          return;
        }
        if (!points || points.length < 2) return;
        const first = points[0];
        const last = points[points.length - 1];
        lastMeasurementSummary.mode = modeDisplayLabel(mode);
        lastMeasurementSummary.modeKey = mode;
        lastMeasurementSummary.timeS = Math.max(0, (Number(last.t_ms || 0) - Number(first.t_ms || 0)) / 1000.0);
        lastMeasurementSummary.distanceM = Math.max(0, Number(distanceM || 0));
        lastMeasurementSummary.startKmh = Number(first.speed_kmh || 0);
        lastMeasurementSummary.endKmh = Number(last.speed_kmh || 0);
        lastMeasurementSummary.status = statusText || 'completed';

        let peakCorrPt = first;
        let peakCorrVal = powerConvert(Number(first.hp_corrected || 0));
        let peakTqPt = first;
        let peakTqVal = Number(first.torque_nm || 0);
        let peakR = 0;
        let maxSp = 0;
        let minSp = 1e9;
        let sumSp = 0;
        let maxThr = 0;
        let maxSlip = 0;
        let sumFuel = 0;
        let nFuel = 0;
        let maxFuel = 0;
        points.forEach(p => {
          const pc = powerConvert(Number(p.hp_corrected || 0));
          if (pc > peakCorrVal) {
            peakCorrVal = pc;
            peakCorrPt = p;
          }
          const tq = Number(p.torque_nm || 0);
          if (tq > peakTqVal) {
            peakTqVal = tq;
            peakTqPt = p;
          }
          peakR = Math.max(peakR, Number(p.rpm || 0));
          const sk = Number(p.speed_kmh || 0);
          maxSp = Math.max(maxSp, sk);
          minSp = Math.min(minSp, sk);
          sumSp += sk;
          maxThr = Math.max(maxThr, Number(p.throttle_pct || 0));
          maxSlip = Math.max(maxSlip, Number(p.slip_pct || 0));
          const fr = Number(p.fuel_rate_lph || 0);
          if (isFinite(fr) && fr >= 0) {
            sumFuel += fr;
            nFuel++;
            maxFuel = Math.max(maxFuel, fr);
          }
        });
        lastMeasurementSummary.peakTorqueAtPeakHp = Number(peakCorrPt.torque_nm || 0);
        lastMeasurementSummary.peakHp = peakCorrVal;
        lastMeasurementSummary.peakHpCorrRpm = Number(peakCorrPt.rpm || 0);
        lastMeasurementSummary.peakTorqueNm = peakTqVal;
        lastMeasurementSummary.finalHp = powerConvert(Number(last.hp_corrected || 0));
        lastMeasurementSummary.finalTorque = Number(last.torque_nm || 0);
        lastMeasurementSummary.peakTorqueRpm = Number(peakTqPt.rpm || 0);
        lastMeasurementSummary.peakRpm = peakR;
        lastMeasurementSummary.maxSpeedKmh = maxSp;
        lastMeasurementSummary.minSpeedKmh = (minSp < 1e8) ? minSp : NaN;
        lastMeasurementSummary.avgSpeedKmh = sumSp / points.length;
        lastMeasurementSummary.startRpm = Number(first.rpm || 0);
        lastMeasurementSummary.endRpm = Number(last.rpm || 0);
        lastMeasurementSummary.maxThrottle = maxThr;
        lastMeasurementSummary.peakHpWheel = powerConvert(Number(peakCorrPt.hp_wheel || 0));
        lastMeasurementSummary.peakHpCrank = powerConvert(Number(peakCorrPt.hp_crank || 0));
        lastMeasurementSummary.peakHpIndicated = powerConvert(Number(peakCorrPt.hp || 0));
        lastMeasurementSummary.peakLossTotal = powerConvert(Number(peakCorrPt.hp_loss_total || 0));
        lastMeasurementSummary.peakLossAero = powerConvert(Number(peakCorrPt.hp_loss_aero || NaN));
        lastMeasurementSummary.peakLossRoll = powerConvert(Number(peakCorrPt.hp_loss_roll || NaN));
        lastMeasurementSummary.peakLossSlope = powerConvert(Number(peakCorrPt.hp_loss_slope || NaN));
        lastMeasurementSummary.maxSlipPct = maxSlip;
        lastMeasurementSummary.avgFuelLph = nFuel > 0 ? (sumFuel / nFuel) : NaN;
        lastMeasurementSummary.maxFuelLph = maxFuel;
        const adFromPt = Number(peakCorrPt.air_density || last.air_density || first.air_density || NaN);
        lastMeasurementSummary.engineOilStartC = Number(first.engine_oil_c || NaN);
        lastMeasurementSummary.engineOilEndC = Number(last.engine_oil_c || NaN);

        const m = envMsg || null;
        if (m) {
          const iat = Number(m.air_intake_c);
          const ph = Number(m.pressure_hpa);
          const hu = Number(m.humidity_pct);
          lastMeasurementSummary.ambientTxt = (isFinite(iat) ? iat.toFixed(1) + ' °C' : '-')
            + ' / ' + (isFinite(ph) ? ph.toFixed(1) + ' hPa' : '-')
            + ' / ' + (isFinite(hu) ? hu.toFixed(0) + ' %RH' : '-');
          const lock = m.gps_lock ? 'LOCK' : 'no lock';
          lastMeasurementSummary.gpsTxt = lock
            + ' | sats ' + Number(m.gps_sats || 0)
            + ' | HDOP ' + Number(m.gps_hdop || 0).toFixed(1)
            + ' | conf ' + Number(gpsStatusSnapshot.confidencePct || 0).toFixed(0) + '%'
            + ' | ' + String(m.gnss_mode || '-')
            + ' | ' + formatRunGpsTelemetryText();
          if (runHasGpsWarning()) {
            lastMeasurementSummary.gpsTxt += ' | WARNING: GNSS quality degraded during run';
          }
          const drv = String(m.drive_type || '-').toUpperCase();
          const gb = Number(m.loss_gearbox_pct || 0);
          const cs = String(m.corr_std || 'DIN').toUpperCase();
          const ck = Number(m.corr_factor_k || 1);
          lastMeasurementSummary.driveType = drv !== '-' ? drv : '';
          lastMeasurementSummary.lossGearboxPct = gb;
          lastMeasurementSummary.corrStd = cs;
          lastMeasurementSummary.corrFactorK = ck;
          lastMeasurementSummary.vehicleTxt = drv + ' | gearbox loss ' + gb.toFixed(1) + '% | ' + cs + ' K=' + ck.toFixed(3);
          const adMsg = Number(m.air_density || NaN);
          lastMeasurementSummary.airDensity = isFinite(adFromPt) ? adFromPt : adMsg;
        } else {
          lastMeasurementSummary.ambientTxt = '-';
          const hadRunGpsWarning = runHasGpsWarning();
          lastMeasurementSummary.gpsTxt = hadRunGpsWarning
            ? ('WARNING: GNSS quality degraded during run | ' + formatRunGpsTelemetryText())
            : ('- | ' + formatRunGpsTelemetryText());
          lastMeasurementSummary.vehicleTxt = '-';
          lastMeasurementSummary.driveType = '';
          lastMeasurementSummary.lossGearboxPct = NaN;
          lastMeasurementSummary.corrStd = '';
          lastMeasurementSummary.corrFactorK = NaN;
          lastMeasurementSummary.airDensity = isFinite(adFromPt) ? adFromPt : NaN;
        }
        renderMeasurementResult();
        lastMainResult = getMainResultString(mode, lastMeasurementSummary);
      }

      function needsStandstillWarmup(mode) {
        return mode === 'drag_0_100' || mode === 'drag_0_200'
          || (mode === 'drag_custom' && parseCustomRange().lo < 15);
      }

      function isRollingThrottleMode(mode) {
        return mode === 'mid_100_200' || mode === 'mid_60_100' || mode === 'mid_80_120' || mode === 'mid_custom'
          || (mode === 'drag_custom' && parseCustomRange().lo >= 15);
      }

      /** Clear committed mode so Measurement menu can pick any mode again (saved result / Results unchanged). */
      function releaseMeasurementModeForNewPick() {
        committedMeasurementMode = '';
        gpsRiskAckForArm = false;
        if (measurementModeEl) measurementModeEl.removeAttribute('data-last');
        resetMeasurementModeSelectDisplay();
        customApplySealActive = false;
        suppressAutoArm = true;
        allowManualStartRun = true;
        refreshModeRequiredStyle();
        updateResultsLayout();
        renderMeasurementResult();
        refreshInteractionLock();
      }

      function refreshMeasurementCloseBtn() {
        const row = document.getElementById('measurementCloseRow');
        if (!row) return;
        const m = getMeasurementMode();
        const show = !!(m && modeNeedsCustomApplyFirst(m) && customApplySealActive && !runArmed && !runActive);
        row.classList.toggle('visible', show);
        row.setAttribute('aria-hidden', show ? 'false' : 'true');
      }

      const MEASUREMENT_MODE_TITLE_IDLE = 'Mode arms automatically when valid';
      function refreshInteractionLock() {
        const lock = runArmed || runActive;
        const modeOk = !!getMeasurementMode();
        if (mainWrap) {
          mainWrap.style.pointerEvents = lock ? 'none' : '';
        }
        if (measurementModeEl) {
          measurementModeEl.disabled = lock || coastCalBlocksControls;
          measurementModeEl.title = lock
            ? 'Cannot change measurement mode while armed or running — ABORT or wait for finish / timeout.'
            : (coastCalBlocksControls
              ? 'Coast-down calibration required — complete calibration or enable bypass in Settings.'
              : MEASUREMENT_MODE_TITLE_IDLE);
        }
        const btnSpApply = document.getElementById('btnCustomSpeedApply');
        const btnDistApply = document.getElementById('btnCustomDistApply');
        if (measurementMenuBlockEl) {
          measurementMenuBlockEl.style.display = runActive ? 'none' : '';
        }
        if (btnSpApply) {
          btnSpApply.disabled = lock || coastCalBlocksControls;
          btnSpApply.title = (lock || coastCalBlocksControls)
            ? (coastCalBlocksControls && !lock ? 'Complete coast-down calibration (or bypass in Settings) before applying.' : 'Unavailable while armed or running')
            : 'Apply speed range to measurement';
        }
        if (btnDistApply) {
          btnDistApply.disabled = lock || coastCalBlocksControls;
          btnDistApply.title = (lock || coastCalBlocksControls)
            ? (coastCalBlocksControls && !lock ? 'Complete coast-down calibration (or bypass in Settings) before applying.' : 'Unavailable while armed or running')
            : 'Apply distance (metres) to measurement';
        }
        const customInputsLocked = lock || customApplySealActive || coastCalBlocksControls;
        const customFieldTitle = !customInputsLocked
          ? null
          : (runActive
            ? 'Cannot edit during a run — ABORT or wait until finished'
            : (runArmed
              ? 'Cannot edit while armed — ABORT to cancel or wait for start/finish'
              : (coastCalBlocksControls
                ? 'Complete coast-down calibration or enable bypass in Settings.'
                : 'Applied — pick another mode and back if you need to change values')));
        if (customStartEl) {
          customStartEl.disabled = customInputsLocked;
          customStartEl.title = customFieldTitle || 'Start speed km/h';
        }
        if (customEndEl) {
          customEndEl.disabled = customInputsLocked;
          customEndEl.title = customFieldTitle || 'End speed km/h';
        }
        if (customDragDistMEl) {
          customDragDistMEl.disabled = customInputsLocked;
          customDragDistMEl.title = customFieldTitle
            || 'Run ends when integrated distance reaches this value (5–5000 m). Use menu presets for ⅛ / ¼ / ½ mile.';
        }
        if (customSpeedPanel) customSpeedPanel.classList.toggle('customFrozen', customInputsLocked);
        if (customDragDistRow) customDragDistRow.classList.toggle('customFrozen', customInputsLocked);
        btnAbort.disabled = !(runArmed || runActive);
        btnAbort.title = (runArmed || runActive) ? 'Abort current/armed run' : 'Nothing to abort';
        if (lock) {
          lockedHint.style.display = 'block';
          lockedHint.textContent = runActive
            ? 'Run in progress — UI locked. Use ABORT to cancel, or wait for finish / timeout.'
            : 'Measurement armed — UI locked. Use ABORT to cancel, or wait for start/finish.';
        } else {
          lockedHint.style.display = 'none';
        }
        refreshStartRunButton();
        updateCustomRangeVisibility();
        refreshMeasurementCloseBtn();
      }

      function escapeHtml(str) {
        return String(str)
          .replace(/&/g, '&amp;')
          .replace(/</g, '&lt;')
          .replace(/>/g, '&gt;')
          .replace(/"/g, '&quot;');
      }

      function showDtModal(message, onOk) {
        dtModalOnOk = typeof onOk === 'function' ? onOk : null;
        if (!dtModal || !dtModalMsg) {
          window.alert(message);
          const fn = dtModalOnOk;
          dtModalOnOk = null;
          if (fn) try { fn(); } catch (e) {}
          return;
        }
        dtModalMsg.textContent = message;
        if (dtModalOk) {
          dtModalOk.disabled = false;
          dtModalOk.textContent = 'OK';
        }
        dtModal.style.display = 'flex';
        dtModal.scrollTop = 0;
        dtModalMsg.scrollTop = 0;
      }
      function showGpsRiskAcknowledgeModal(onConfirm) {
        const modeLabel = modeDisplayLabel(getMeasurementMode() || '');
        const satsTxt = Number(gpsStatusSnapshot.sats || 0).toFixed(0);
        const hdopTxt = Number(gpsStatusSnapshot.hdop || 99).toFixed(1);
        const confTxt = Number(gpsStatusSnapshot.confidencePct || 0).toFixed(0);
        const lockTxt = gpsStatusSnapshot.lock ? 'LOCK' : 'NO LOCK';
        const gnssTxt = String(gpsStatusSnapshot.gnssMode || 'NONE');
        const htmlMsg = ''
          + '<b>GPS quality warning</b><br><br>'
          + 'Current GNSS quality is not ideal for precise road measurement in <b>' + escapeHtml(modeLabel || 'this mode') + '</b>.<br>'
          + 'Status: <b>' + escapeHtml(lockTxt) + '</b> | sats <b>' + escapeHtml(satsTxt) + '</b> | HDOP <b>' + escapeHtml(hdopTxt) + '</b> | Confidence <b>' + escapeHtml(confTxt) + '%</b> | ' + escapeHtml(gnssTxt) + '<br><br>'
          + 'M9N/N9M class receivers can use up to 4 main constellations at once (GPS + GLONASS + Galileo + BeiDou) while tracking many satellites in total. '
          + 'SBAS/QZSS are assist systems, not primary timing constellations.<br><br>'
          + 'Measurement can continue, but result accuracy may be reduced. For best accuracy, place the unit under the windshield with clear sky view and wait for stable lock.<br><br>'
          + '<label style="display:flex;gap:8px;align-items:flex-start;line-height:1.35;">'
          + '<input id="gpsRiskAckChk" type="checkbox" style="margin-top:2px;">'
          + '<span>I have read this warning and I accept measuring at my own responsibility.</span>'
          + '</label>';
        dtModalOnOk = typeof onConfirm === 'function' ? onConfirm : null;
        if (!dtModal || !dtModalMsg || !dtModalOk) {
          const ok = window.confirm(
            'GPS quality warning: lock/sats/HDOP are not ideal. Results may be less accurate. Continue at your own responsibility?'
          );
          if (ok && dtModalOnOk) {
            const fn = dtModalOnOk;
            dtModalOnOk = null;
            try { fn(); } catch (e) {}
          }
          return;
        }
        dtModalMsg.innerHTML = htmlMsg;
        dtModalOk.textContent = 'I UNDERSTAND - CONTINUE';
        dtModalOk.disabled = true;
        dtModal.style.display = 'flex';
        dtModal.scrollTop = 0;
        dtModalMsg.scrollTop = 0;
        const chk = document.getElementById('gpsRiskAckChk');
        if (chk) chk.addEventListener('change', () => { dtModalOk.disabled = !chk.checked; });
      }
      function hideDtModal() {
        if (dtModal) dtModal.style.display = 'none';
        if (dtModalMsg) dtModalMsg.textContent = '';
        if (dtModalOk) {
          dtModalOk.disabled = false;
          dtModalOk.textContent = 'OK';
        }
        const fn = dtModalOnOk;
        dtModalOnOk = null;
        if (fn) try { fn(); } catch (e) {}
      }
      if (dtModalOk) dtModalOk.addEventListener('click', hideDtModal);
      if (dtModalBackdrop) dtModalBackdrop.addEventListener('click', hideDtModal);

      function refreshStartRunButton() {
        if (!btnStartRun) return;
        btnStartRun.classList.remove('btnStartRun--armed', 'btnStartRun--running');
        const m = getMeasurementMode();
        const trackCanArm = m === '__track_nav__' && trackSessionActive && !runArmed && !runActive;
        if (runActive) {
          btnStartRun.classList.add('btnStartRun--running');
          btnStartRun.setAttribute('aria-pressed', 'true');
          btnStartRun.disabled = true;
          btnStartRun.textContent = 'RUNNING';
          btnStartRun.title = '';
        } else if (runArmed) {
          btnStartRun.classList.add('btnStartRun--armed');
          btnStartRun.setAttribute('aria-pressed', 'true');
          btnStartRun.disabled = true;
          btnStartRun.textContent = 'ARMED';
          btnStartRun.title = '';
        } else {
          btnStartRun.setAttribute('aria-pressed', 'false');
          btnStartRun.disabled = coastCalBlocksControls || (!allowManualStartRun && !trackCanArm);
          btnStartRun.textContent = 'START RUN';
          btnStartRun.title = coastCalBlocksControls
            ? 'Complete coast-down calibration first, or enable bypass in Settings.'
            : (trackCanArm && !allowManualStartRun
              ? ('Press START RUN on the line to save the gate. Lap timing starts when speed exceeds '
                + cachedAutoArmKmh.toFixed(0) + ' km/h (Auto-arm GPS in Settings). Finish by returning to the gate and stopping (~'
                + TRACK_STANDSTILL_KMH + ' km/h or less).')
              : (allowManualStartRun
                ? (measurementAutoArmPref
                  ? ''
                  : 'Press START RUN when you are ready to arm. The run still uses the same mode start rules (speed / throttle / RPM) after you arm.')
                : (measurementAutoArmPref
                  ? 'Measurement arms when you pick a mode, then starts automatically when conditions are met. START RUN also unlocks after a finished run, timeout, or ABORT — or use it in Track mode as above.'
                  : 'Choose a mode, then press START RUN to arm. The run still starts from the same mode thresholds. Track: use START RUN on the line as usual.')));
        }
      }

      /** Phone / small touch: defer gauge canvas paint until after home layout (avoids main-thread stall when leaving Results). */
      function isMobileTouchNarrowUi() {
        try {
          if (typeof window.matchMedia !== 'function' || !window.matchMedia('(max-width: 1024px)').matches) return false;
          const tp = (typeof navigator.maxTouchPoints === 'number') ? navigator.maxTouchPoints : 0;
          if (tp > 0) return true;
          return 'ontouchstart' in window;
        } catch (e) { return false; }
      }

      /** When a run starts, scroll the dashboard so the analog gauge block is clearly in view. */
      function scrollHomeToAnalogGauges() {
        if (activeScreen !== 'home') return;
        const el = document.getElementById('analogGaugesAnchor');
        if (!el) return;
        requestAnimationFrame(() => {
          requestAnimationFrame(() => {
            try {
              el.scrollIntoView({ behavior: 'smooth', block: 'center', inline: 'nearest' });
            } catch (e) {}
          });
        });
      }

      function setScreen(screen) {
        if (runActive || runArmed) return;
        const prevScreen = activeScreen;
        if (screen === 'home') {
          dynoResultsRedrawToken++;
        }
        activeScreen = screen;
        if (screen === 'home' && isMobileTouchNarrowUi() && prevScreen === 'results') {
          isSwitchingToHome = true;
        }
        document.querySelectorAll('.screenBlock').forEach(el => {
          el.classList.toggle('active', el.getAttribute('data-screen') === screen);
        });
        if (subNavDashBack) {
          subNavDashBack.style.display = (screen === 'results') ? 'flex' : 'none';
        }
        if (btnResultsScreen) btnResultsScreen.classList.toggle('active', screen === 'results');

        const paintGaugesAfterSwitch = () => {
          if (lastLiveMsg) {
            paintHomeCardsFromMsg(lastLiveMsg);
            paintMeasurementGauges(lastLiveMsg);
          } else {
            drawLiveGauges(0, 0, 0, 0);
          }
          isSwitchingToHome = false;
        };
        /* Desktop: single rAF. Mobile: double rAF so layout commits before painting 4 gauge canvases.
           (Removed 300ms delay + full-screen "Loading..." overlay on Results→Home — it felt like a ~2s stall.) */
        if (screen === 'home' && isMobileTouchNarrowUi()) {
          requestAnimationFrame(() => {
            requestAnimationFrame(paintGaugesAfterSwitch);
          });
        } else {
          requestAnimationFrame(paintGaugesAfterSwitch);
        }

        if (screen === 'home') {
          applyCustomModeArmPolicy(getMeasurementMode());
          refreshInteractionLock();
        } else {
          if (screen === 'results') {
            if (getMeasurementMode() === '__track_nav__') {
              syncTrackSessionToMeasurementSummary();
            }
            updateResultsLayout();
            refreshDynoResultsCanvas();
          }
          updateCustomRangeVisibility();
        }
      }

      function haversineMeters(lat1, lon1, lat2, lon2) {
        const toRad = Math.PI / 180.0;
        const dLat = (lat2 - lat1) * toRad;
        const dLon = (lon2 - lon1) * toRad;
        const a = Math.sin(dLat * 0.5) * Math.sin(dLat * 0.5)
          + Math.cos(lat1 * toRad) * Math.cos(lat2 * toRad) * Math.sin(dLon * 0.5) * Math.sin(dLon * 0.5);
        const c = 2.0 * Math.atan2(Math.sqrt(a), Math.sqrt(1.0 - a));
        return 6371000.0 * c;
      }

      function resetTrackCurrentLapStats() {
        trackCurrentTopSpeed = 0;
        trackCurrentSpeedSum = 0;
        trackCurrentSamples = 0;
        trackCurrentRpmTrend = [];
        trackCurrentSpeedTrend = [];
        trackCurrentHpTrend = [];
      }

      function clearTrackStartRunArmState() {
        trackStartRunLapArm = false;
        trackGateLat = null;
        trackGateLon = null;
        trackAwaitingSpeedForLap = false;
        trackRunHasLeftGate = false;
      }

      function renderTrackLaps() {
        if (!trackLapList) return;
        if (!trackLaps.length) {
          trackLapList.textContent = 'No laps yet.';
          return;
        }
        trackLapList.innerHTML = trackLaps.map(l => {
          const mark = l.best_lap ? ' ⭐' : '';
          return '<div class="runItem">Lap ' + l.lap_number + ': ' + l.time.toFixed(2) + ' s | top '
            + l.top_speed.toFixed(1) + ' km/h | avg ' + l.avg_speed.toFixed(1) + ' km/h' + mark + '</div>';
        }).join('');
      }

      function updateTrackSmartHint() {
        const el = document.getElementById('trackSmartHint');
        if (!el) return;
        if (!trackSessionActive) {
          el.textContent = 'Session stopped. Open Track again from the menu to continue, or use Reset session to clear laps.';
          return;
        }
        if (trackStartRunLapArm) {
          el.textContent = runActive
            ? ('START RUN lap: return to the saved line and stop (≤' + TRACK_STANDSTILL_KMH + ' km/h) to finish. Next lap: exceed ~' + cachedAutoArmKmh + ' km/h again from the line.')
            : ('ARMED — gate saved. Lap timing starts when speed exceeds ~' + cachedAutoArmKmh + ' km/h (Auto-arm GPS in Settings). Finish: return to gate and stop.');
          return;
        }
        if (trackOriginLat == null || trackOriginLon == null) {
          el.textContent = 'With GPS lock, stop at your start/finish line below ~' + TRACK_STANDSTILL_KMH + ' km/h — the line is saved automatically. Then leave at least ~' + TRACK_MIN_LEAVE_M + ' m; every return through the line completes a lap at any speed. Further laps: drive away from the line and cross again — no stop required.';
        } else if (!trackHasLeftOrigin) {
          el.textContent = 'Line locked — drive at least ~' + TRACK_MIN_LEAVE_M + ' m away from the line, then cross it again to finish lap ' + trackCurrentLapNo + '.';
        } else {
          el.textContent = 'Timing lap ' + trackCurrentLapNo + '… Cross the line to finish. Next lap starts from that crossing — keep circulating until you end the session.';
        }
      }

      function resetTrackLineAnchor() {
        trackOriginLat = null;
        trackOriginLon = null;
        trackStartLat = null;
        trackStartLon = null;
        trackMaxDistFromOriginM = 0;
        trackHasLeftOrigin = false;
        trackLapInZone = false;
        trackLapStartTms = 0;
        trackCurrentLapNo = trackLaps.length > 0 ? (trackLaps.length + 1) : 1;
        resetTrackCurrentLapStats();
      }

      function beginTrackPanelSessionAndScroll() {
        trackSessionActive = true;
        resetTrackLineAnchor();
        updateTrackHeader();
        renderTrackLaps();
        requestAnimationFrame(() => {
          requestAnimationFrame(() => {
            if (trackModePanel) {
              try {
                trackModePanel.scrollIntoView({ behavior: 'smooth', block: 'center', inline: 'nearest' });
              } catch (e) {}
            }
          });
        });
      }

      function updateTrackHeader() {
        if (trackSessionInfo) trackSessionInfo.textContent = 'Session: ' + (trackSessionActive ? 'live' : 'stopped');
        if (trackLapInfo) trackLapInfo.textContent = 'Lap: ' + (trackCurrentLapNo > 0 ? String(trackCurrentLapNo) : '-');
        if (trackBestInfo) trackBestInfo.textContent = 'Best: ' + (trackBestLapS > 0 ? (trackBestLapS.toFixed(2) + ' s') : '-');
        if (btnTrackStart) btnTrackStart.disabled = false;
        if (btnTrackStop) btnTrackStop.disabled = !trackSessionActive;
        updateTrackSmartHint();
      }

      function restoreSummarySnapshotIfAny() {
        if (!lastMeasurementSummarySnapshotBeforeTrack) return;
        try {
          const o = JSON.parse(lastMeasurementSummarySnapshotBeforeTrack);
          Object.assign(lastMeasurementSummary, o);
        } catch (e) {}
        lastMeasurementSummarySnapshotBeforeTrack = null;
      }

      function syncTrackSessionToMeasurementSummary() {
        if (getMeasurementMode() !== '__track_nav__') return;
        const n = trackLaps.length;
        const last = n ? trackLaps[n - 1] : null;
        let sumT = 0;
        let sumAvgSp = 0;
        let maxTop = 0;
        let minTop = 1e9;
        trackLaps.forEach(l => {
          sumT += l.time;
          sumAvgSp += l.avg_speed;
          if (l.top_speed > maxTop) maxTop = l.top_speed;
          if (l.top_speed < minTop) minTop = l.top_speed;
        });
        const avgLap = n > 0 ? (sumT / n) : NaN;
        const avgOfLapAvgs = n > 0 ? (sumAvgSp / n) : NaN;
        lastMeasurementSummary.modeKey = '__track_nav__';
        lastMeasurementSummary.mode = modeDisplayLabel('__track_nav__');
        lastMeasurementSummary.status = (trackSessionActive ? 'Session live' : 'Session stopped')
          + (n ? (' · ' + n + ' lap' + (n === 1 ? '' : 's') + ' recorded') : ' · no laps yet');
        lastMeasurementSummary.timeS = (trackBestLapS > 0) ? trackBestLapS : NaN;
        lastMeasurementSummary.distanceM = NaN;
        lastMeasurementSummary.startKmh = NaN;
        lastMeasurementSummary.endKmh = NaN;
        lastMeasurementSummary.avgSpeedKmh = avgOfLapAvgs;
        lastMeasurementSummary.maxSpeedKmh = n > 0 ? maxTop : NaN;
        lastMeasurementSummary.minSpeedKmh = (n > 0 && minTop < 1e8) ? minTop : NaN;
        lastMeasurementSummary.peakHp = NaN;
        lastMeasurementSummary.peakHpCorrRpm = NaN;
        lastMeasurementSummary.peakTorqueNm = NaN;
        lastMeasurementSummary.peakTorqueAtPeakHp = NaN;
        lastMeasurementSummary.finalHp = NaN;
        lastMeasurementSummary.finalTorque = NaN;
        lastMeasurementSummary.peakTorqueRpm = NaN;
        lastMeasurementSummary.peakRpm = NaN;
        lastMeasurementSummary.startRpm = NaN;
        lastMeasurementSummary.endRpm = NaN;
        lastMeasurementSummary.maxThrottle = NaN;
        lastMeasurementSummary.peakHpWheel = NaN;
        lastMeasurementSummary.peakHpCrank = NaN;
        lastMeasurementSummary.peakHpIndicated = NaN;
        lastMeasurementSummary.peakLossTotal = NaN;
        lastMeasurementSummary.peakLossAero = NaN;
        lastMeasurementSummary.peakLossRoll = NaN;
        lastMeasurementSummary.peakLossSlope = NaN;
        lastMeasurementSummary.maxSlipPct = NaN;
        lastMeasurementSummary.avgFuelLph = NaN;
        lastMeasurementSummary.maxFuelLph = NaN;
        if (lastLiveMsg) {
          const m = lastLiveMsg;
          const iat = Number(m.air_intake_c);
          const ph = Number(m.pressure_hpa);
          const hu = Number(m.humidity_pct);
          lastMeasurementSummary.ambientTxt = (isFinite(iat) ? iat.toFixed(1) + ' °C' : '-')
            + ' / ' + (isFinite(ph) ? ph.toFixed(1) + ' hPa' : '-')
            + ' / ' + (isFinite(hu) ? hu.toFixed(0) + ' %RH' : '-');
          const lock = m.gps_lock ? 'LOCK' : 'no lock';
          lastMeasurementSummary.gpsTxt = lock
            + ' | sats ' + Number(m.gps_sats || 0)
            + ' | HDOP ' + Number(m.gps_hdop || 0).toFixed(1)
            + ' | conf ' + Number(gpsStatusSnapshot.confidencePct || 0).toFixed(0) + '%'
            + ' | ' + String(m.gnss_mode || '-');
          if (runGpsSamples > 0) {
            lastMeasurementSummary.gpsTxt += ' | ' + formatRunGpsTelemetryText();
          }
          if (runHasGpsWarning()) {
            lastMeasurementSummary.gpsTxt += ' | WARNING: GNSS quality degraded during run';
          }
          const drv = String(m.drive_type || '-').toUpperCase();
          const gb = Number(m.loss_gearbox_pct || 0);
          const cs = String(m.corr_std || 'DIN').toUpperCase();
          const ck = Number(m.corr_factor_k || 1);
          lastMeasurementSummary.driveType = drv !== '-' ? drv : '';
          lastMeasurementSummary.lossGearboxPct = gb;
          lastMeasurementSummary.corrStd = cs;
          lastMeasurementSummary.corrFactorK = ck;
          lastMeasurementSummary.vehicleTxt = drv + ' | gearbox loss ' + gb.toFixed(1) + '% | ' + cs + ' K=' + ck.toFixed(3);
          const adFromPt = Number(m.air_density || NaN);
          lastMeasurementSummary.airDensity = isFinite(adFromPt) ? adFromPt : NaN;
        }
        renderMeasurementResult();
      }

      function enterCommittedTrackMode() {
        if (committedMeasurementMode !== '__track_nav__') {
          measurementModeBeforeTrack = committedMeasurementMode || '';
          try {
            lastMeasurementSummarySnapshotBeforeTrack = JSON.stringify(lastMeasurementSummary);
          } catch (e) {
            lastMeasurementSummarySnapshotBeforeTrack = null;
          }
        }
        committedMeasurementMode = '__track_nav__';
        if (measurementModeEl) measurementModeEl.setAttribute('data-last', '__track_nav__');
        customApplySealActive = false;
        clearTrackStartRunArmState();
        disarmIfWaitingOnly();
        suppressAutoArm = true;
        syncTrackSessionToMeasurementSummary();
      }

      function seedIdleMeasurementSummary(modeKey) {
        lastMeasurementSummary.modeKey = modeKey;
        lastMeasurementSummary.mode = modeDisplayLabel(modeKey);
        lastMeasurementSummary.status = 'idle';
        lastMeasurementSummary.timeS = NaN;
        lastMeasurementSummary.distanceM = NaN;
        lastMeasurementSummary.startKmh = NaN;
        lastMeasurementSummary.endKmh = NaN;
        lastMeasurementSummary.avgSpeedKmh = NaN;
        lastMeasurementSummary.maxSpeedKmh = NaN;
        lastMeasurementSummary.minSpeedKmh = NaN;
        lastMeasurementSummary.peakHp = NaN;
        lastMeasurementSummary.peakHpCorrRpm = NaN;
        lastMeasurementSummary.peakTorqueNm = NaN;
        lastMeasurementSummary.peakTorqueAtPeakHp = NaN;
        lastMeasurementSummary.finalHp = NaN;
        lastMeasurementSummary.finalTorque = NaN;
        lastMeasurementSummary.peakTorqueRpm = NaN;
        lastMeasurementSummary.peakRpm = NaN;
        lastMeasurementSummary.startRpm = NaN;
        lastMeasurementSummary.endRpm = NaN;
        lastMeasurementSummary.maxThrottle = NaN;
        lastMeasurementSummary.peakHpWheel = NaN;
        lastMeasurementSummary.peakHpCrank = NaN;
        lastMeasurementSummary.peakHpIndicated = NaN;
        lastMeasurementSummary.peakLossTotal = NaN;
        lastMeasurementSummary.peakLossAero = NaN;
        lastMeasurementSummary.peakLossRoll = NaN;
        lastMeasurementSummary.peakLossSlope = NaN;
        lastMeasurementSummary.maxSlipPct = NaN;
        lastMeasurementSummary.avgFuelLph = NaN;
        lastMeasurementSummary.maxFuelLph = NaN;
        lastMeasurementSummary.engineOilStartC = NaN;
        lastMeasurementSummary.engineOilEndC = NaN;
        lastMeasurementSummary.airDensity = NaN;
        lastMeasurementSummary.corrStd = '';
        lastMeasurementSummary.corrFactorK = NaN;
        lastMeasurementSummary.lossGearboxPct = NaN;
        lastMeasurementSummary.driveType = '';
        lastMeasurementSummary.ambientTxt = '-';
        lastMeasurementSummary.gpsTxt = '-';
        lastMeasurementSummary.vehicleTxt = '-';
      }

      function clearMeasurementSummaryNoMode() {
        lastMeasurementSummary.modeKey = '-';
        lastMeasurementSummary.mode = '-';
        lastMeasurementSummary.status = 'idle';
        seedIdleMeasurementSummary('');
        lastMeasurementSummary.modeKey = '-';
        lastMeasurementSummary.mode = '-';
      }

      function leaveCommittedTrackModeToMenuChoice(pickedMode) {
        if (committedMeasurementMode !== '__track_nav__') return;
        if (pickedMode === measurementModeBeforeTrack) {
          restoreSummarySnapshotIfAny();
        } else {
          lastMeasurementSummarySnapshotBeforeTrack = null;
          if (pickedMode) seedIdleMeasurementSummary(pickedMode);
          else clearMeasurementSummaryNoMode();
        }
        measurementModeBeforeTrack = '';
      }

      function updateRotateHint() {
        const isSmall = window.matchMedia('(max-width: 900px)').matches;
        const portrait = window.matchMedia('(orientation: portrait)').matches;
        rotateHint.style.display = (isSmall && portrait) ? 'flex' : 'none';
      }

      function updateSetup(msg) {
        if (msg.setup_ok) {
          lastMissingFields = [];
          setupBanner.style.display = 'none';
          return;
        }
        const missing = Array.isArray(msg.missing_fields) ? msg.missing_fields : [];
        lastMissingFields = missing.slice();
        const weightMissing = missing.includes('weightKg');
        if (setupLink) setupLink.href = weightMissing ? '/settings?focus=weightKg' : '/settings';
        if (weightMissing && missing.length === 1) {
          setupTitle.textContent = 'Enter vehicle weight to continue';
          setupMsg.textContent = '';
        } else if (missing.length > 0) {
          setupTitle.textContent = 'Setup incomplete - fill required fields';
          setupMsg.textContent = '';
        } else {
          setupTitle.textContent = 'Complete setup to start measurement';
          setupMsg.textContent = '';
        }
        setupBanner.style.display = 'block';
      }

      function updateCoastCal(msg) {
        const bypass = !!(msg.coast_bypass || msg.coastBypass);
        const valid = !!(msg.coast_cal_valid || msg.coastCalValid);
        coastCalBlocksControls = !bypass && !valid;
        const conf = Number(msg.coast_cal_conf || msg.coastCalConf || 0);
        const reason = String(msg.coast_cal_reason || msg.coastCalReason || 'repeat procedure');
        if (elCoastCalInfo) {
          elCoastCalInfo.classList.remove('health-normal', 'health-warn', 'health-hot');
          if (bypass) {
            elCoastCalInfo.classList.add('health-warn');
            elCoastCalInfo.textContent = 'CoastCal: bypass enabled';
            if (calBanner) calBanner.style.display = 'none';
            return;
          }
          if (valid) {
            elCoastCalInfo.classList.add('health-normal');
            elCoastCalInfo.textContent = 'CoastCal: valid (' + conf.toFixed(0) + '%)';
          } else {
            elCoastCalInfo.classList.add('health-hot');
            elCoastCalInfo.textContent = 'CoastCal: invalid - repeat (' + conf.toFixed(0) + '%)';
          }
        }
        if (calBanner) {
          if (valid) {
            calBanner.style.display = 'none';
          } else {
            if (calMsg) calMsg.textContent = 'Repeat coast-down calibration. Reason: ' + reason;
            calBanner.style.display = 'block';
          }
        }
        if (btnCalibrate) btnCalibrate.disabled = valid;
        if (coastCalBlocksControls && runArmed && !runActive) {
          disarmIfWaitingOnly();
        }
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

      let cueAudioCtx = null;
      function beep(freq = 920, ms = 120, gainPeak = 0.12) {
        try {
          if (!cueAudioCtx) cueAudioCtx = new (window.AudioContext || window.webkitAudioContext)();
          const ac = cueAudioCtx;
          if (ac.state === 'suspended' && typeof ac.resume === 'function') {
            ac.resume().catch(() => {});
          }
          const o = ac.createOscillator();
          const g = ac.createGain();
          o.frequency.value = freq;
          o.connect(g);
          g.connect(ac.destination);
          const gp = Math.max(0.04, Math.min(0.35, Number(gainPeak) || 0.12));
          const now = ac.currentTime;
          g.gain.setValueAtTime(0.0001, now);
          g.gain.exponentialRampToValueAtTime(gp, now + 0.02);
          g.gain.exponentialRampToValueAtTime(0.0001, now + Math.max(0.05, ms / 1000));
          o.start();
          setTimeout(() => { try { o.stop(); } catch (e) {} }, ms);
        } catch (e) {}
      }

      function gnssBadgeLevelFromReadiness(gpsConfidencePct, gpsMainReady) {
        if (gpsConfidencePct >= 78 && gpsMainReady) return 'green';
        if (gpsConfidencePct >= 50) return 'yellow';
        return 'red';
      }

      function playGnssYellowWarn() {
        // Caution: clearly audible but softer than critical — two rising mid tones (red stays low/grave).
        beep(680, 165, 0.2);
        setTimeout(() => beep(820, 175, 0.2), 230);
      }

      function playGnssRedCritical() {
        beep(380, 280, 0.36);
        setTimeout(() => beep(300, 360, 0.36), 320);
      }

      /**
       * Badge-tier sounds when idle/armed. During a road run, badge audio is off (GNSS jitter); critical alerts use isGpsWeakForActiveRun + cooldown.
       * Track lap run: yellow muted; red tier still uses cooldown to limit spam.
       */
      function updateGnssBadgeAlertSounds(prev, next, opts) {
        opts = opts || {};
        const silenceBadgeDuringRoadRun = !!opts.silenceBadgeDuringRoadRun;
        const suppressYellowDuringRun = !!opts.suppressYellowDuringRun;

        if (silenceBadgeDuringRoadRun) {
          if (prev === null) {
            gnssBadgeAlertPrevLevel = next;
            return;
          }
          if (prev === next) return;
          if (next === 'green') {
            gnssBadgeAlertPrevLevel = next;
            return;
          }
          gnssBadgeAlertPrevLevel = next;
          return;
        }

        if (prev === null) {
          if (next === 'green') {
            gnssBadgeAlertPrevLevel = next;
            return;
          }
          if (next === 'yellow') {
            if (!suppressYellowDuringRun) playGnssYellowWarn();
          } else if (next === 'red') {
            playGnssRedCritical();
            lastGnssRedCriticalPlayMs = Date.now();
          }
          gnssBadgeAlertPrevLevel = next;
          return;
        }
        if (prev === next) return;
        if (next === 'green') {
          gnssBadgeAlertPrevLevel = next;
          return;
        }
        if (next === 'yellow' && prev !== 'yellow') {
          if (!suppressYellowDuringRun) playGnssYellowWarn();
        } else if (next === 'red' && prev !== 'red') {
          const nowMs = Date.now();
          const allowRed = (prev === 'green') || (nowMs - lastGnssRedCriticalPlayMs >= GNSS_RED_ALERT_COOLDOWN_MS);
          if (allowRed) {
            playGnssRedCritical();
            lastGnssRedCriticalPlayMs = nowMs;
          }
        }
        gnssBadgeAlertPrevLevel = next;
      }

      function drawAnalogGauge(canvasEl, value, min, max, title, unit, color, opts) {
        opts = opts || {};
        if (!canvasEl) return;
        const c = canvasEl.getContext('2d');

        // Crisp rendering on high-DPI screens + stable drawing even with responsive CSS.
        const dpr = window.devicePixelRatio || 1;
        const rect = canvasEl.getBoundingClientRect();
        const attrW = parseInt(canvasEl.getAttribute('width'), 10) || 340;
        const attrH = parseInt(canvasEl.getAttribute('height'), 10) || 180;
        let w = rect.width;
        let h = rect.height;
        if (w < 16 || h < 16) {
          w = attrW;
          h = attrH;
        }

        const targetW = Math.max(1, Math.round(w * dpr));
        const targetH = Math.max(1, Math.round(h * dpr));
        if (canvasEl.width !== targetW) canvasEl.width = targetW;
        if (canvasEl.height !== targetH) canvasEl.height = targetH;

        c.setTransform(dpr, 0, 0, dpr, 0, 0);
        c.clearRect(0, 0, w, h);
        c.fillStyle = '#000000';
        c.fillRect(0, 0, w, h);

        const cx = w * 0.5;
        const cy = h * 0.86;
        const r = Math.min(w * 0.42, h * 0.72);
        const start = Math.PI * 0.85;
        const end = Math.PI * 2.15;

        const labelFontPx = Math.max(10, Math.round(h * 0.055));
        const titleFontPx = Math.max(11, Math.round(h * 0.06));
        const bigFontPx = Math.max(24, Math.round(h * 0.19));

        c.lineCap = 'round';
        c.lineWidth = 12;
        c.strokeStyle = 'rgba(255,255,255,0.22)';
        c.beginPath();
        c.arc(cx, cy, r, start, end, false);
        c.stroke();
        const drawZone = (fromVal, toVal, zoneColor) => {
          const f = Math.max(0, Math.min(1, (fromVal - min) / (max - min || 1)));
          const t = Math.max(0, Math.min(1, (toVal - min) / (max - min || 1)));
          if (t <= f) return;
          c.strokeStyle = zoneColor;
          c.lineWidth = 14;
          c.beginPath();
          c.arc(cx, cy, r, start + f * (end - start), start + t * (end - start), false);
          c.stroke();
        };
        if (isFinite(opts.warnFrom)) drawZone(opts.warnFrom, isFinite(opts.dangerFrom) ? opts.dangerFrom : max, 'rgba(255,210,90,0.95)');
        if (isFinite(opts.dangerFrom)) drawZone(opts.dangerFrom, max, 'rgba(255,90,90,0.95)');

        const steps = 8;
        c.lineWidth = 3;
        c.strokeStyle = 'rgba(255,255,255,0.45)';
        c.fillStyle = 'rgba(255,255,255,0.98)';
        c.font = 'bold ' + labelFontPx + 'px Arial';
        for (let i = 0; i <= steps; i++) {
          const t = i / steps;
          const a = start + t * (end - start);
          const x1 = cx + Math.cos(a) * (r - 2);
          const y1 = cy + Math.sin(a) * (r - 2);
          const x2 = cx + Math.cos(a) * (r - 14);
          const y2 = cy + Math.sin(a) * (r - 14);
          c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
          const v = min + t * (max - min);
          const tx = cx + Math.cos(a) * (r - 28);
          const ty = cy + Math.sin(a) * (r - 28);
          c.shadowColor = 'rgba(0,0,0,0.65)';
          c.shadowBlur = 2;
          c.fillText(String(Math.round(v)), tx - 8, ty + 3);
          c.shadowBlur = 0;
        }

        const clamped = Math.max(min, Math.min(max, value));
        const p = (clamped - min) / (max - min || 1);
        const ang = start + p * (end - start);
        c.lineWidth = 7;
        c.strokeStyle = color;
        c.beginPath();
        c.moveTo(cx, cy);
        c.lineTo(cx + Math.cos(ang) * (r - 22), cy + Math.sin(ang) * (r - 22));
        c.stroke();
        c.fillStyle = color;
        c.beginPath();
        c.arc(cx, cy, 7, 0, Math.PI * 2);
        c.fill();
        if (isFinite(opts.peak) && opts.peak > min) {
          const pp = Math.max(0, Math.min(1, (opts.peak - min) / (max - min || 1)));
          const pa = start + pp * (end - start);
          const px = cx + Math.cos(pa) * (r - 6);
          const py = cy + Math.sin(pa) * (r - 6);
          c.fillStyle = '#ffffff';
          c.beginPath();
          c.arc(px, py, 4, 0, Math.PI * 2);
          c.fill();
        }

        c.fillStyle = '#dce9ff';
        c.font = 'bold ' + titleFontPx + 'px Arial';
        c.shadowColor = 'rgba(0,0,0,0.75)';
        c.shadowBlur = 3;
        c.fillText(title, 10, Math.round(titleFontPx * 1.35));

        c.font = 'bold ' + bigFontPx + 'px Arial';
        const bigX = cx - Math.round(bigFontPx * 0.78);
        const bigY = h - Math.round(bigFontPx * 0.14);
        c.fillText(Math.round(clamped) + ' ' + unit, bigX, bigY);
        c.shadowBlur = 0;
      }

      function drawLiveGauges(speedKmh, rpmVal, powerVal, torqueVal) {
        drawAnalogGauge(gaugeSpeed, speedKmh, 0, 260, 'SPEED', 'km/h', '#00ff88');
        drawAnalogGauge(gaugeRpm, rpmVal, 0, 8000, 'RPM', 'rpm', '#ffd400', {
          warnFrom: redlineRpm * 0.9,
          dangerFrom: redlineRpm
        });
        const pUnit = powerUnitLabel();
        const pMax = pUnit === 'kw' ? 450 : 600;
        // `powerVal` is based on hp_corrected => app-estimated power at CRANK.
        drawAnalogGauge(gaugePower, powerVal, 0, pMax, 'POWER @ CRANK', pUnit, '#7db2ff', {
          warnFrom: sessionPeakPower > 0 ? sessionPeakPower * 0.85 : NaN,
          dangerFrom: sessionPeakPower > 0 ? sessionPeakPower * 0.95 : NaN,
          peak: sessionPeakPower
        });
        drawAnalogGauge(gaugeTorque, torqueVal, 0, 1000, 'TORQUE', 'Nm', '#ff8aa0', {
          warnFrom: sessionPeakTorque > 0 ? sessionPeakTorque * 0.85 : NaN,
          dangerFrom: sessionPeakTorque > 0 ? sessionPeakTorque * 0.95 : NaN,
          peak: sessionPeakTorque
        });
      }

      function vibrate(pattern) {
        try {
          if (navigator && typeof navigator.vibrate === 'function') navigator.vibrate(pattern);
        } catch (e) {}
      }

      function flash(color, ms = 180) {
        flashOverlay.style.background = color;
        flashOverlay.style.opacity = '0.34';
        setTimeout(() => { flashOverlay.style.opacity = '0'; }, ms);
      }

      /** Full-screen red tint (like GO green / FINISH yellow cues) — uses flashOverlay only (pointer-events: none, never blocks UI). */
      function flashAbortScreen() {
        if (!flashOverlay) return;
        const prevZ = flashOverlay.style.zIndex;
        flashOverlay.style.zIndex = '10004';
        flashOverlay.style.background = '#c01010';
        flashOverlay.style.opacity = '0.82';
        const ms = 420;
        setTimeout(() => {
          flashOverlay.style.opacity = '0';
          setTimeout(() => {
            flashOverlay.style.zIndex = prevZ || '';
            flashOverlay.style.background = '';
          }, 200);
        }, ms);
      }

      function hideRunCue() {
        const el = document.getElementById('runCueOverlay');
        if (runCueHideTimer) {
          clearTimeout(runCueHideTimer);
          runCueHideTimer = null;
        }
        if (!el) return;
        el.classList.remove('visible', 'runCueOverlay--go', 'runCueOverlay--finish', 'runCueOverlay--result', 'runCueOverlay--abort');
        el.setAttribute('aria-hidden', 'true');
        const t = document.getElementById('runCueOverlayText');
        if (t) t.textContent = '';
      }

      /** Full-screen cue + Web Speech API (where supported). kind: 'go' | 'finish' | 'abort' */
      function showRunCue(kind, durationMs) {
        const el = document.getElementById('runCueOverlay');
        const txtEl = document.getElementById('runCueOverlayText');
        const hintEl = document.getElementById('runCueOverlayHint');
        if (!el || !txtEl) return;
        hideRunCue();
        let word, speak;
        if (kind === 'finish') {
          word = 'FINISH';
          speak = 'finish';
        } else if (kind === 'abort') {
          word = 'ABORT';
          speak = 'abort';
        } else {
          word = 'GO';
          speak = 'GO';
        }
        el.classList.remove('runCueOverlay--go', 'runCueOverlay--finish', 'runCueOverlay--result', 'runCueOverlay--abort');
        el.classList.add('runCueOverlay--' + kind);
        txtEl.textContent = word;
        if (hintEl) hintEl.textContent = 'Tap to dismiss';
        el.classList.add('visible');
        el.setAttribute('aria-hidden', 'false');
        // Beep instead of speech
        if (kind === 'go') {
          beep(1240, 500);
        } else if (kind === 'finish') {
          beep(1240, 1000);
        } else if (kind === 'abort') {
          beep(1240, 200);
          setTimeout(() => beep(1240, 200), 300);
          setTimeout(() => beep(1240, 200), 600);
        }
        const ms = typeof durationMs === 'number' && durationMs > 0 ? durationMs : (kind === 'finish' ? 2000 : 1400);
        runCueHideTimer = setTimeout(() => {
          if (kind === 'finish' && lastMainResult) {
            txtEl.textContent = lastMainResult;
            el.classList.remove('runCueOverlay--finish');
            el.classList.add('runCueOverlay--result');
            // clear the timer since it stays
            runCueHideTimer = null;
          } else {
            hideRunCue();
          }
        }, ms);
      }

      (function wireRunCueOverlay() {
        const el = document.getElementById('runCueOverlay');
        if (!el) return;
        const dismiss = () => hideRunCue();
        el.addEventListener('click', dismiss);
        el.addEventListener('touchstart', dismiss, { passive: true });
      })();

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
        /* Saved runs list UI removed; persistRuns still stores last 25 in localStorage. */
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

      /** RPM-bin average + light MA — smooth dyno curves in 1500…redline band. */
      function buildDynoPlotSeries(sortedRaw, rMin, rMax) {
        const span = rMax - rMin;
        if (span < 100 || !sortedRaw.length) return [];
        const nBins = Math.min(128, Math.max(32, Math.floor(sortedRaw.length * 0.5)));
        const bins = [];
        for (let b = 0; b < nBins; b++) {
          const lo = rMin + (b / nBins) * span;
          const hi = rMin + ((b + 1) / nBins) * span;
          let n = 0, hpS = 0, tqS = 0, lossS = 0, rpmS = 0;
          for (let k = 0; k < sortedRaw.length; k++) {
            const p = sortedRaw[k];
            const r = Number(p.rpm) || 0;
            const inBin = b === nBins - 1 ? (r >= lo && r <= rMax) : (r >= lo && r < hi);
            if (!inBin) continue;
            n++;
            hpS += powerConvert(p.hp_corrected || 0);
            tqS += Number(p.torque_nm || 0);
            lossS += powerConvert(p.hp_loss_total || 0);
            rpmS += r;
          }
          const rpm = n > 0 ? rpmS / n : (lo + hi) / 2;
          const hp = n > 0 ? hpS / n : 0;
          const tq = n > 0 ? tqS / n : 0;
          const loss = n > 0 ? lossS / n : 0;
          bins.push({ rpm, hp, tq, loss, n });
        }
        // Interpolate missing bins
        const validIndices = [];
        for (let i = 0; i < bins.length; i++) {
          if (bins[i].n > 0) validIndices.push(i);
        }
        for (let i = 0; i < bins.length; i++) {
          if (bins[i].n > 0) continue;
          const leftIdx = validIndices.filter(v => v < i).pop();
          const rightIdx = validIndices.find(v => v > i);
          if (leftIdx != null && rightIdx != null) {
            const frac = (bins[i].rpm - bins[leftIdx].rpm) / (bins[rightIdx].rpm - bins[leftIdx].rpm);
            bins[i].hp = bins[leftIdx].hp + frac * (bins[rightIdx].hp - bins[leftIdx].hp);
            bins[i].tq = bins[leftIdx].tq + frac * (bins[rightIdx].tq - bins[leftIdx].tq);
            bins[i].loss = bins[leftIdx].loss + frac * (bins[rightIdx].loss - bins[leftIdx].loss);
          } else if (leftIdx != null) {
            bins[i].hp = bins[leftIdx].hp;
            bins[i].tq = bins[leftIdx].tq;
            bins[i].loss = bins[leftIdx].loss;
          } else if (rightIdx != null) {
            bins[i].hp = bins[rightIdx].hp;
            bins[i].tq = bins[rightIdx].tq;
            bins[i].loss = bins[rightIdx].loss;
          }
        }
        if (bins.length < 2) {
          return sortedRaw.map((p) => ({
            rpm: Number(p.rpm) || 0,
            hp: powerConvert(p.hp_corrected || 0),
            tq: Number(p.torque_nm || 0),
            loss: powerConvert(p.hp_loss_total || 0)
          })).filter((o) => o.rpm >= rMin && o.rpm <= rMax);
        }
        function ma9(arr, key) {
          const out = arr.map((x) => x[key]);
          if (arr.length < 9) return out;
          return arr.map((_, i) => {
            const vals = [];
            for (let j = Math.max(0, i - 4); j <= Math.min(arr.length - 1, i + 4); j++) {
              vals.push(out[j]);
            }
            return vals.reduce((a, b) => a + b, 0) / vals.length;
          });
        }
        const hpM = ma9(bins, 'hp');
        const tqM = ma9(bins, 'tq');
        const lossM = ma9(bins, 'loss');
        return bins.map((row, i) => ({ rpm: row.rpm, hp: hpM[i], tq: tqM[i], loss: lossM[i] }));
      }

      /** Optional fixedLogical: { w, h, dpr } for off-screen export / print (skips layout rect). */
      function drawCurveOn(cv, points, fixedLogical) {
        const c = cv.getContext('2d');
        let w, h, dpr;
        if (fixedLogical && fixedLogical.w > 0 && fixedLogical.h > 0) {
          w = fixedLogical.w;
          h = fixedLogical.h;
          dpr = fixedLogical.dpr != null ? fixedLogical.dpr : (window.devicePixelRatio || 1);
        } else {
          dpr = window.devicePixelRatio || 1;
          const rect = cv.getBoundingClientRect();
          w = rect.width || cv.width / dpr;
          h = rect.height || cv.height / dpr;
        }
        const targetW = Math.max(1, Math.round(w * dpr));
        const targetH = Math.max(1, Math.round(h * dpr));
        if (cv.width !== targetW) cv.width = targetW;
        if (cv.height !== targetH) cv.height = targetH;

        c.setTransform(dpr, 0, 0, dpr, 0, 0);
        c.clearRect(0, 0, w, h);
        c.fillStyle = '#060606';
        c.fillRect(0, 0, w, h);
        if (!points.length) return;

        const rLine = Math.max(4000, Math.min(9200, Number(redlineRpm) || 6500));
        const minR = 1500;
        const maxR = rLine;
        const sortedAll = points.slice().sort((a, b) => (Number(a.rpm) || 0) - (Number(b.rpm) || 0));
        let sorted = sortedAll.filter((p) => {
          const r = Number(p.rpm) || 0;
          return r >= minR && r <= maxR;
        });
        if (sorted.length < 3) sorted = sortedAll;

        let plot = buildDynoPlotSeries(sorted, minR, maxR);
        if (plot.length < 2) {
          plot = sorted.map((p) => ({
            rpm: Number(p.rpm) || 0,
            hp: powerConvert(p.hp_corrected || 0),
            tq: Number(p.torque_nm || 0),
            loss: powerConvert(p.hp_loss_total || 0)
          })).filter((o) => o.rpm >= minR && o.rpm <= maxR);
        }
        if (plot.length < 2) {
          plot = sorted.map((p) => ({
            rpm: Number(p.rpm) || 0,
            hp: powerConvert(p.hp_corrected || 0),
            tq: Number(p.torque_nm || 0),
            loss: powerConvert(p.hp_loss_total || 0)
          }));
        }
        if (plot.length < 2) return;

        const left = 64, right = w - 64, top = 14;
        const bottom = h - 48;
        let maxHp = 1, maxTq = 1, maxLoss = 1;
        plot.forEach((row) => {
          maxHp = Math.max(maxHp, row.hp);
          maxTq = Math.max(maxTq, row.tq);
          maxLoss = Math.max(maxLoss, row.loss);
        });
        maxHp = Math.ceil(maxHp / 10) * 10;
        maxTq = Math.ceil(maxTq / 10) * 10;
        maxLoss = Math.ceil(maxLoss / 10) * 10;
        const maxPowerAxis = Math.max(maxHp, maxLoss);
        const mapX = r => ((Math.min(maxR, Math.max(minR, r)) - minR) / (maxR - minR)) * (right - left) + left;
        const mapYHp = hpV => bottom - (hpV / maxPowerAxis) * (bottom - top);
        const mapYLoss = hpV => bottom - (hpV / maxPowerAxis) * (bottom - top);
        const mapYTq = t => bottom - (t / maxTq) * (bottom - top);

        c.strokeStyle = 'rgba(255,255,255,0.12)';
        c.fillStyle = 'rgba(220,232,255,0.90)';
        c.font = '12px Arial';
        c.shadowColor = 'rgba(0,0,0,0.55)';
        c.shadowBlur = 2;
        const rpmSpan = maxR - minR;
        const tickStep = rpmSpan <= 3500 ? 500 : 1000;
        for (let rpmTick = Math.ceil(minR / tickStep) * tickStep; rpmTick <= maxR + 0.1; rpmTick += tickStep) {
          if (rpmTick < minR) continue;
          const x = mapX(rpmTick);
          c.beginPath(); c.moveTo(x, top); c.lineTo(x, bottom); c.stroke();
          c.fillText(String(Math.round(rpmTick)), x - (rpmTick >= 10000 ? 16 : 12), bottom + 12);
        }
        c.shadowBlur = 0;
        const yTicks = 6;
        for (let i = 0; i <= yTicks; i++) {
          const y = top + ((bottom - top) * i) / yTicks;
          c.beginPath(); c.moveTo(left, y); c.lineTo(right, y); c.stroke();
          const leftVal = ((maxPowerAxis * (yTicks - i)) / yTicks);
          const rightVal = ((maxTq * (yTicks - i)) / yTicks);
          c.textAlign = 'right';
          c.fillText(leftVal.toFixed(0), left - 10, y + 3);
          c.textAlign = 'left';
          c.fillText(rightVal.toFixed(0), right + 10, y + 3);
        }

        c.strokeStyle = 'rgba(255,255,255,0.28)';
        c.lineWidth = 1.2;
        c.beginPath(); c.moveTo(left, top); c.lineTo(left, bottom); c.lineTo(right, bottom); c.stroke();
        c.beginPath(); c.moveTo(right, top); c.lineTo(right, bottom); c.stroke();
        const axisUnitFont = 'bold 13px Arial';
        c.font = axisUnitFont;
        c.fillStyle = '#00ffa0';
        c.textAlign = 'left';
        c.textBaseline = 'alphabetic';
        c.fillText(powerUnit === 'kw' ? 'kW' : 'HP', 2, 22);
        c.fillStyle = '#7db2ff';
        c.textAlign = 'right';
        c.fillText('Nm', w - 4, 22);
        c.fillStyle = 'rgba(220,232,255,0.92)';
        c.textAlign = 'center';
        c.textBaseline = 'alphabetic';
        c.fillText('RPM', w * 0.5, h - 3);
        c.textAlign = 'left';
        c.fillStyle = '#00ffa0';
        c.fillText('Power', left + 4, top + 12);
        c.fillStyle = '#7db2ff';
        c.fillText('Torque', left + 52, top + 12);
        c.fillStyle = '#ff8aa0';
        c.fillText('Loss', left + 108, top + 12);

        c.lineWidth = 2.4;
        c.lineJoin = 'round';
        c.lineCap = 'round';
        c.strokeStyle = '#00ffa0';
        c.beginPath();
        plot.forEach((row, i) => {
          const x = mapX(row.rpm), y = mapYHp(row.hp);
          if (i === 0) c.moveTo(x, y); else c.lineTo(x, y);
        });
        c.stroke();
        c.strokeStyle = '#7db2ff';
        c.beginPath();
        plot.forEach((row, i) => {
          const x = mapX(row.rpm), y = mapYTq(row.tq);
          if (i === 0) c.moveTo(x, y); else c.lineTo(x, y);
        });
        c.stroke();
        c.strokeStyle = '#ff8aa0';
        c.beginPath();
        plot.forEach((row, i) => {
          const x = mapX(row.rpm), y = mapYLoss(row.loss);
          if (i === 0) c.moveTo(x, y); else c.lineTo(x, y);
        });
        c.stroke();
      }

      function drawCurve(points) {
        drawCurveOn(canvas, points, null);
      }

      function updateReportFromPoints(points, msg) {
        if (!points.length) return;
        const u = powerUnitLabel();
        let peakHp = { v: 0, rpm: 0, p: null };
        let peakTq = { v: 0, rpm: 0 };
        let maxSlip = 0;
        let sumFuel = 0;
        let nFuel = 0;
        let maxFuel = 0;
        points.forEach(p => {
          const pc = powerConvert(p.hp_corrected || 0);
          if (pc > peakHp.v) peakHp = { v: pc, rpm: p.rpm || 0, p };
          if ((p.torque_nm || 0) > peakTq.v) peakTq = { v: p.torque_nm || 0, rpm: p.rpm || 0 };
          maxSlip = Math.max(maxSlip, Number(p.slip_pct || 0));
          const fr = Number(p.fuel_rate_lph || 0);
          if (isFinite(fr) && fr >= 0) {
            sumFuel += fr;
            nFuel++;
            maxFuel = Math.max(maxFuel, fr);
          }
        });
        const peakP = peakHp.p || points[points.length - 1];
        peakPowerInfo.textContent = peakHp.v.toFixed(1) + ' ' + u + ' @ ' + peakHp.rpm.toFixed(0) + ' RPM';
        peakTorqueInfo.textContent = peakTq.v.toFixed(1) + ' Nm @ ' + peakTq.rpm.toFixed(0) + ' RPM';
        wheelVsEngineInfo.textContent = powerConvert(peakP.hp_wheel || 0).toFixed(1) + ' ' + u
          + ' wheel / ' + powerConvert(peakP.hp_crank || 0).toFixed(1) + ' ' + u + ' engine';
        const lt = powerConvert(Number(peakP.hp_loss_total || 0));
        const la = powerConvert(Number(peakP.hp_loss_aero || NaN));
        const lr = powerConvert(Number(peakP.hp_loss_roll || NaN));
        const ls = powerConvert(Number(peakP.hp_loss_slope || NaN));
        if (dynoLossAtPeakInfo) {
          let t = lt.toFixed(1) + ' ' + u + ' total';
          if (isFinite(la) || isFinite(lr) || isFinite(ls)) {
            t += ' (' + (isFinite(la) ? la.toFixed(1) : '-') + ' aero · ' + (isFinite(lr) ? lr.toFixed(1) : '-') + ' roll · '
              + (isFinite(ls) ? ls.toFixed(1) : '-') + ' slope ' + u + ')';
          }
          dynoLossAtPeakInfo.textContent = t;
        }
        driveLossInfo.textContent = Number(msg.loss_gearbox_pct || 0).toFixed(1) + '%';
        corrSummaryInfo.textContent = String(msg.corr_std || 'DIN').toUpperCase() + ' | K=' + Number(msg.corr_factor_k || 1).toFixed(3);
        ambientInfo.textContent = Number(msg.air_intake_c || 0).toFixed(1) + ' C, '
          + Number(msg.pressure_hpa || 1013.2).toFixed(1) + ' hPa, '
          + Number(msg.humidity_pct || 0).toFixed(1) + ' %';
        if (dynoSlipFuelInfo) {
          const slipTxt = maxSlip.toFixed(1) + ' %';
          const fuelTxt = nFuel > 0
            ? (sumFuel / nFuel).toFixed(2) + ' · ' + maxFuel.toFixed(2) + ' L/h'
            : '-';
          dynoSlipFuelInfo.textContent = slipTxt + ' / ' + fuelTxt;
        }
      }

      function saveCurrentRun(modeKeyOpt, opts) {
        if (!currentRun.length) return;
        const mk = modeKeyOpt || getMeasurementMode();
        savedRuns.push({
          startedAt: new Date().toISOString(),
          points: currentRun.slice(),
          projectType: 'custom',
          projectTag: '',
          mode: mk
        });
        persistRuns();
        renderRuns();
        const peak = currentRun.reduce((m, p) => Math.max(m, powerConvert(p.hp_corrected || 0)), 0);
        // Removed sound: if (!opts || !opts.quietVoice) { say('Run saved. Peak power ' + peak.toFixed(0) + ' ' + powerUnitLabel() + '.'); }
        // Removed beep: beep(1040, 120);
      }

      function abortRun(reason) {
        const hadActive = runActive;
        const modeBeforeAbort = getMeasurementMode();
        const pointsBeforeAbort = currentRun.slice();
        const distBeforeAbort = runDistanceM;
        runActive = false;
        runArmed = false;
        gpsRiskAckForArm = false;
        customApplySealActive = false;
        clearTrackStartRunArmState();
        runReady = false;
        suppressAutoArm = true;
        runDistanceM = 0;
        runDistancePrevM = 0;
        runStartTms = 0;
        runDisplayStartPerf = 0;
        runLastSpeedKmh = 0;
        runLastSpeedChangeTms = 0;
        prevRunSample = null;
        hideRunCue();
        flashAbortScreen();
        /* Do not use showRunCue('abort') — that overlay has pointer-events:auto and blocks taps ~1.4s. */
        beep(1240, 200);
        setTimeout(() => beep(1240, 200), 300);
        setTimeout(() => beep(1240, 200), 600);
        if (btnAbort) {
          btnAbort.disabled = true;
          btnAbort.title = 'Nothing to abort';
        }
        refreshInteractionLock();

        elSpeed.textContent = '0';
        elRpm.textContent = '0';
        elHp.textContent = '0';
        elTorque.textContent = '0';
        elHpWheel.textContent = '0';
        elHpCrank.textContent = '0';
        elHpCorr.textContent = '0';
        elThrottle.textContent = '0';
        elFuelRate.textContent = '0.0';
        drawLiveGauges(0, 0, 0, 0);
        msPowerWheel.textContent = '0';
        msG.textContent = '0.00';
        msRunTime.textContent = '0.00';
        msRunDistance.textContent = '0.0';
        elAutoRunInfo.textContent = 'AutoRun: aborted';
        elAutoRunReasonInfo.textContent = 'Reason: ' + reason;
        if (hadActive) {
          vibrate([220, 80, 120]);
        }
        currentRun = [];
        drawCurve([]);

        function finishAbortHeavy() {
          if (hadActive && modeBeforeAbort && modeBeforeAbort !== '__track_nav__') {
            setMeasurementResultFromRun(modeBeforeAbort, pointsBeforeAbort, distBeforeAbort, 'aborted', lastLiveMsg);
          }
          runGpsDropDetected = false;
          runGpsDropNotified = false;
          lastRunHadGpsDrop = false;
          resetRunGpsTelemetry();
          allowManualStartRun = true;
          refreshInteractionLock();
          updateResultsLayout();
          refreshDynoResultsCanvas();
        }
        if (typeof requestAnimationFrame === 'function') {
          requestAnimationFrame(finishAbortHeavy);
        } else {
          finishAbortHeavy();
        }
      }

      function csvCell(v) {
        const s = String(v == null ? '' : v);
        if (/[",\n\r]/.test(s)) return '"' + s.replace(/"/g, '""') + '"';
        return s;
      }
      function csvLine(fields) {
        return fields.map((x) => csvCell(x)).join(',');
      }

      /** Sample rows for EXPORT: same run as on-screen / Print Report (not all savedRuns). */
      function getPointsForExportedRun() {
        const s = lastMeasurementSummary;
        if (!s.modeKey || s.modeKey === '-' || !s.status || s.status === 'idle') return [];
        if (s.status === 'aborted') return [];
        if (s.modeKey === '__track_nav__') return [];
        const uiMode = getMeasurementMode();
        if (currentRun.length >= 2) {
          if (uiMode === s.modeKey) return currentRun;
          if (s.modeKey === 'dyno_pull' && !uiMode) return currentRun;
        }
        for (let i = savedRuns.length - 1; i >= 0; i--) {
          const r = savedRuns[i];
          const pts = r.points || [];
          if ((r.mode || '') === s.modeKey && pts.length >= 2) return pts;
        }
        return [];
      }

      /** Same logical fields / wording as Print Report (label, display value) for CSV mirror block. */
      function buildReportPrintMirrorCsvRows(s, u, lastLiveMsg) {
        const rows = [];
        const hasRun = s.modeKey && s.modeKey !== '-' && s.status && s.status !== 'idle';
        if (!hasRun) return rows;
        if (s.status === 'aborted') {
          rows.push(['Note', 'Run aborted — no valid measurement values.']);
          rows.push(['Mode', s.mode || '-']);
          rows.push(['Status', 'Aborted']);
          return rows;
        }
        const skipDistance = s.modeKey === 'dyno_pull';
        const isDynoRun = s.modeKey === 'dyno_pull';
        const timeTxt = isFinite(s.timeS) ? (s.timeS.toFixed(2) + ' s') : '-';
        const distTxt = isFinite(s.distanceM) ? (s.distanceM.toFixed(1) + ' m') : '-';
        const speedWindowTxt = (isFinite(s.startKmh) && isFinite(s.endKmh))
          ? (s.startKmh.toFixed(1) + ' → ' + s.endKmh.toFixed(1) + ' km/h') : '-';
        const spdTxt = (isFinite(s.avgSpeedKmh) && isFinite(s.maxSpeedKmh))
          ? (s.avgSpeedKmh.toFixed(1) + ' avg / ' + s.maxSpeedKmh.toFixed(1) + ' max / '
            + (isFinite(s.minSpeedKmh) ? s.minSpeedKmh.toFixed(1) : '-') + ' min km/h') : '-';
        let peakPowerTxt = '-';
        if (isFinite(s.peakHp)) {
          peakPowerTxt = isFinite(s.peakHpCorrRpm)
            ? (s.peakHp.toFixed(1) + ' ' + u + ' (corr.) @ ' + Math.round(s.peakHpCorrRpm) + ' rpm')
            : (s.peakHp.toFixed(1) + ' ' + u + ' (corr.)');
        }
        let peakTorqueTxt = '-';
        if (isFinite(s.peakTorqueNm)) {
          peakTorqueTxt = isFinite(s.peakTorqueRpm)
            ? (s.peakTorqueNm.toFixed(1) + ' Nm @ ' + Math.round(s.peakTorqueRpm) + ' rpm')
            : (s.peakTorqueNm.toFixed(1) + ' Nm');
        }
        const peakRpmTxt = isFinite(s.peakRpm) ? (String(Math.round(s.peakRpm)) + ' rpm') : '-';
        const rpmWinTxt = (isFinite(s.startRpm) && isFinite(s.endRpm))
          ? (Math.round(s.startRpm) + ' → ' + Math.round(s.endRpm) + ' rpm') : '-';
        const maxThrTxt = isFinite(s.maxThrottle) ? (s.maxThrottle.toFixed(0) + ' %') : '-';
        const pf = (v) => (isFinite(v) ? (v.toFixed(1) + ' ' + u) : '-');
        const pwrSplitTxt = (pf(s.peakHpWheel) !== '-' || pf(s.peakHpCrank) !== '-' || pf(s.peakHpIndicated) !== '-')
          ? (pf(s.peakHpWheel) + ' / ' + pf(s.peakHpCrank) + ' / ' + pf(s.peakHpIndicated)) : '-';
        let lossTxt = '-';
        if (isFinite(s.peakLossTotal)) {
          lossTxt = pf(s.peakLossTotal) + ' total';
          if (isFinite(s.peakLossAero) || isFinite(s.peakLossRoll) || isFinite(s.peakLossSlope)) {
            lossTxt += ' (' + pf(s.peakLossAero) + ' aero, ' + pf(s.peakLossRoll) + ' roll, ' + pf(s.peakLossSlope) + ' slope)';
          }
        }
        const slipTxt = isFinite(s.maxSlipPct) ? (s.maxSlipPct.toFixed(1) + ' %') : '-';
        const fuelTxt = (isFinite(s.avgFuelLph) && isFinite(s.maxFuelLph))
          ? (s.avgFuelLph.toFixed(2) + ' / ' + s.maxFuelLph.toFixed(2) + ' L/h') : '-';
        const oilTxt = (isFinite(s.engineOilStartC) && isFinite(s.engineOilEndC))
          ? (s.engineOilStartC.toFixed(1) + ' → ' + s.engineOilEndC.toFixed(1) + ' °C') : '-';
        const rhoTxt = isFinite(s.airDensity) ? (s.airDensity.toFixed(4) + ' kg/m³') : '-';
        let corrTxt = '-';
        if (s.corrStd && isFinite(s.corrFactorK)) corrTxt = String(s.corrStd).toUpperCase() + ' | K=' + s.corrFactorK.toFixed(3);
        else if (s.corrStd) corrTxt = String(s.corrStd).toUpperCase();
        else if (isFinite(s.corrFactorK)) corrTxt = 'K=' + s.corrFactorK.toFixed(3);
        const drvP = s.driveType ? String(s.driveType).toUpperCase() : '';
        const gbP = isFinite(s.lossGearboxPct) ? (s.lossGearboxPct.toFixed(1) + '%') : '';
        let vehTxt = '-';
        if (drvP && gbP) vehTxt = drvP + ' | gearbox loss ' + gbP;
        else if (drvP) vehTxt = drvP;
        else if (s.vehicleTxt && s.vehicleTxt !== '-') vehTxt = s.vehicleTxt;

        if (s.modeKey === '__track_nav__') {
          const nL = trackLaps.length;
          let sumT = 0;
          trackLaps.forEach(l => { sumT += l.time; });
          const avgLStr = nL > 0 ? ((sumT / nL).toFixed(2) + ' s') : '—';
          const lastL = nL ? trackLaps[nL - 1] : null;
          const lastStr = lastL ? (lastL.time.toFixed(2) + ' s') : '—';
          const bestStr = trackBestLapS > 0 ? (trackBestLapS.toFixed(2) + ' s') : '—';
          const timeTxtTrack = isFinite(s.timeS) ? (s.timeS.toFixed(2) + ' s (best lap)') : '—';
          const spdTrack = (isFinite(s.avgSpeedKmh) && isFinite(s.maxSpeedKmh))
            ? ('Across laps: ' + s.avgSpeedKmh.toFixed(1) + ' avg / ' + s.maxSpeedKmh.toFixed(1) + ' max / '
              + (isFinite(s.minSpeedKmh) ? s.minSpeedKmh.toFixed(1) : '-') + ' min km/h (per-lap top speeds)') : '—';
          rows.push(['Mode', s.mode || '-']);
          rows.push(['Status', s.status || '-']);
          rows.push(['Laps recorded', String(nL)]);
          rows.push(['Best lap', bestStr]);
          rows.push(['Average lap', avgLStr]);
          rows.push(['Last lap', lastStr]);
          rows.push(['Duration (best lap)', timeTxtTrack]);
          rows.push(['Speed stats (laps)', spdTrack]);
          rows.push(['Ambient (IAT / P / RH)', s.ambientTxt || '-']);
          rows.push(['GPS', s.gpsTxt || '-']);
          rows.push(['Drive / gearbox loss', vehTxt]);
          rows.push(['Note (track)', 'Per-lap table: use EXPORT TRACK CSV in the track panel.']);
          return rows;
        }

        rows.push(['Mode', s.mode || '-']);
        rows.push(['Status', s.status || '-']);
        rows.push(['Duration', timeTxt]);
        if (!skipDistance) rows.push(['Distance', distTxt]);
        rows.push(['Speed window (start → end)', speedWindowTxt]);
        rows.push(['Avg / max / min speed', spdTxt]);
        rows.push(['Peak power (corr.) @ RPM', peakPowerTxt]);
        rows.push(['Peak torque @ RPM', peakTorqueTxt]);
        rows.push(['Max RPM (during run)', peakRpmTxt]);
        rows.push(['RPM (start → end)', rpmWinTxt]);
        rows.push(['Max throttle', maxThrTxt]);
        rows.push(['Power @ peak corr. (WHP / crank / indicated)', pwrSplitTxt]);
        rows.push(['Road-load losses @ peak corr.', lossTxt]);
        rows.push(['Max tire slip', slipTxt]);
        rows.push(['Fuel rate (avg / max)', fuelTxt]);
        rows.push(['Engine oil (start → end)', oilTxt]);
        rows.push(['Air density', rhoTxt]);
        rows.push(['Correction (std / K)', corrTxt]);
        rows.push(['Ambient (IAT / P / RH)', s.ambientTxt || '-']);
        rows.push(['GPS', s.gpsTxt || '-']);
        rows.push(['Drive / gearbox loss', vehTxt]);

        if (isDynoRun) {
          const dynoPeakP = (isFinite(s.peakHp) && isFinite(s.peakHpCorrRpm))
            ? (s.peakHp.toFixed(1) + ' ' + u + ' @ ' + Math.round(s.peakHpCorrRpm) + ' rpm') : '-';
          const dynoPeakT = (isFinite(s.peakTorqueNm) && isFinite(s.peakTorqueRpm))
            ? (s.peakTorqueNm.toFixed(1) + ' Nm @ ' + Math.round(s.peakTorqueRpm) + ' rpm') : '-';
          const dynoWhp = pwrSplitTxt !== '-' ? pwrSplitTxt : '-';
          const dynoGb = isFinite(s.lossGearboxPct) ? (s.lossGearboxPct.toFixed(1) + '%') : '-';
          const dynoSlipFuel = slipTxt + ' / ' + (fuelTxt === '-' ? '—' : fuelTxt);
          rows.push(['_print_section', 'Dyno graph summary']);
          rows.push(['Peak power (corr.) @ RPM', dynoPeakP]);
          rows.push(['Peak torque @ RPM', dynoPeakT]);
          rows.push(['WHP vs engine @ peak corr.', dynoWhp]);
          rows.push(['Loss @ peak corr.', lossTxt]);
          rows.push(['Gearbox loss', dynoGb]);
          rows.push(['Correction', corrTxt]);
          rows.push(['Max slip / fuel (avg · max)', dynoSlipFuel]);
          rows.push(['Ambient (T/P/H)', s.ambientTxt || '-']);
          rows.push(['Note (dyno)', 'Crank power is estimated from drivetrain and road-load losses.']);
        }

        if (lastLiveMsg && (lastLiveMsg.t_ms !== undefined || lastLiveMsg.speed_kmh !== undefined)) {
          const lm = lastLiveMsg;
          rows.push(['_print_section', 'Last live sample (reference)']);
          rows.push(['Speed (fused)', Number(lm.speed_kmh || 0).toFixed(1) + ' km/h']);
          rows.push(['Speed OBD / GPS', Number(lm.speed_obd_kmh || 0).toFixed(1) + ' / ' + Number(lm.speed_gps_kmh || 0).toFixed(1) + ' km/h']);
          rows.push(['RPM', String(Math.round(Number(lm.rpm || 0)))]);
          rows.push(['Throttle', Number(lm.throttle_pct || 0).toFixed(0) + ' %']);
          rows.push(['Power corrected / torque', powerConvert(Number(lm.hp_corrected || 0)).toFixed(1) + ' ' + u + ' / ' + Number(lm.torque_nm || 0).toFixed(0) + ' Nm']);
        }

        rows.push(['Note (report)', 'Power/torque figures use the correction selected in settings. Road modes use GPS/OBD fusion as shown in the live app.']);
        return rows;
      }

      function exportRunsCsv() {
        const u = powerUnitLabel();
        const s = lastMeasurementSummary;
        const hasReport = s.modeKey && s.modeKey !== '-' && s.status && s.status !== 'idle';
        if (!hasReport) {
          showDtModal('No measurement result yet. Open Track for a live session (then export), or finish a drag / rolling / braking / dyno run — export matches Print Report for the current result.');
          return;
        }
        if (s.status === 'aborted') {
          const lines = [];
          lines.push(csvLine(['export_format_version', '3']));
          lines.push(csvLine(['generated_utc', new Date().toISOString()]));
          lines.push(csvLine(['power_unit', u]));
          lines.push(csvLine(['torque_unit', 'Nm']));
          lines.push(csvLine(['speed_unit', 'km/h']));
          lines.push(csvLine(['summary_mode_key', s.modeKey || '']));
          lines.push(csvLine(['note', 'Run aborted — no valid measurement values.']));
          lines.push('');
          const wideH = [
            'mode_key', 'mode_label', 'status', 'time_s', 'distance_m',
            'speed_start_kmh', 'speed_end_kmh', 'speed_avg_kmh', 'speed_max_kmh', 'speed_min_kmh',
            'rpm_start', 'rpm_end', 'rpm_peak', 'rpm_at_peak_power', 'rpm_at_peak_torque', 'max_throttle_pct',
            'power_peak_corrected', 'torque_peak_nm', 'power_peak_wheel', 'power_peak_crank', 'power_peak_indicated',
            'loss_peak_total', 'loss_peak_aero', 'loss_peak_roll', 'loss_peak_slope',
            'max_slip_pct', 'fuel_avg_lph', 'fuel_max_lph',
            'oil_temp_start_c', 'oil_temp_end_c', 'air_density_kgm3',
            'corr_std', 'corr_factor_k', 'drive_type', 'gearbox_loss_pct',
            'ambient', 'gps', 'vehicle_summary'
          ];
          const wideV = [s.modeKey || '', s.mode || '', 'aborted'].concat(Array(35).fill(''));
          lines.push(csvLine(['block', 'last_run_summary']));
          lines.push(csvLine(wideH));
          lines.push(csvLine(wideV));
          lines.push('');
          lines.push(csvLine(['block', 'report_print_mirror']));
          lines.push(csvLine(['label', 'value']));
          buildReportPrintMirrorCsvRows(s, u, lastLiveMsg).forEach((pair) => {
            lines.push(csvLine(pair));
          });
          lines.push('');
          lines.push(csvLine(['block', 'time_series']));
          lines.push(csvLine(['note', 'No samples — run aborted.']));
          const out = '\uFEFF' + lines.join('\n');
          const blob = new Blob([out], { type: 'text/csv;charset=utf-8' });
          const a = document.createElement('a');
          a.href = URL.createObjectURL(blob);
          const safeMk = String(s.modeKey || 'run').replace(/[^a-z0-9_-]/gi, '_');
          const isoShort = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
          a.download = 'dynotrack_export_' + safeMk + '_' + isoShort + '.csv';
          a.click();
          setTimeout(() => { try { URL.revokeObjectURL(a.href); } catch (e) {} }, 2500);
          return;
        }
        function nStr(v, dec) {
          const x = Number(v);
          if (!isFinite(x)) return '';
          return dec != null ? x.toFixed(dec) : String(Math.round(x));
        }
        function pStr(v, dec) {
          const x = Number(v);
          if (!isFinite(x)) return '';
          const c = powerConvert(x);
          return dec != null ? c.toFixed(dec) : String(c);
        }
        const tsHeader = [
          'run_id', 'mode_key', 'sample_index', 't_ms', 'speed_kmh', 'rpm', 'throttle_pct', 'torque_nm',
          'power_indicated', 'power_crank', 'power_corrected', 'power_wheel',
          'loss_total', 'loss_aero', 'loss_roll', 'loss_slope',
          'fuel_lph', 'iat_c', 'oil_temp_c', 'slip_pct', 'air_density_kgm3', 'anomaly'
        ];
        const sampleRows = [];
        function pushPoints(runIdx, modeStr, points) {
          const modeLabel = modeStr || '-';
          points.forEach((p, i) => {
            const nb = (x) => (isFinite(Number(x)) ? Number(x) : '');
            sampleRows.push(csvLine([
              runIdx,
              modeLabel,
              i,
              p.t_ms,
              nb(p.speed_kmh) !== '' ? nStr(p.speed_kmh, 2) : '',
              nb(p.rpm) !== '' ? nStr(p.rpm, 0) : '',
              nb(p.throttle_pct) !== '' ? nStr(p.throttle_pct, 1) : '',
              nb(p.torque_nm) !== '' ? nStr(p.torque_nm, 1) : '',
              pStr(p.hp, 2),
              pStr(p.hp_crank, 2),
              pStr(p.hp_corrected, 2),
              pStr(p.hp_wheel, 2),
              pStr(p.hp_loss_total, 2),
              pStr(p.hp_loss_aero, 2),
              pStr(p.hp_loss_roll, 2),
              pStr(p.hp_loss_slope, 2),
              nb(p.fuel_rate_lph) !== '' ? nStr(p.fuel_rate_lph, 3) : '',
              nb(p.air_intake_c) !== '' ? nStr(p.air_intake_c, 1) : '',
              nb(p.engine_oil_c) !== '' ? nStr(p.engine_oil_c, 1) : '',
              nb(p.slip_pct) !== '' ? nStr(p.slip_pct, 2) : '',
              nb(p.air_density) !== '' ? nStr(p.air_density, 5) : '',
              p.anomaly || ''
            ]));
          });
        }
        const exportPoints = getPointsForExportedRun();
        if (exportPoints.length >= 2) {
          pushPoints(1, s.modeKey, exportPoints);
        }
        const hasSamples = sampleRows.length > 0;
        const lines = [];
        lines.push(csvLine(['export_format_version', '3']));
        lines.push(csvLine(['generated_utc', new Date().toISOString()]));
        lines.push(csvLine(['power_unit', u]));
        lines.push(csvLine(['torque_unit', 'Nm']));
        lines.push(csvLine(['speed_unit', 'km/h']));
        lines.push(csvLine(['summary_mode_key', s.modeKey || '']));
        lines.push(csvLine(['note', 'Aligned with Print Report: one measurement only. Time series = samples for this run (run_id=1).']));
        lines.push('');
        const wideH = [
          'mode_key', 'mode_label', 'status', 'time_s', 'distance_m',
          'speed_start_kmh', 'speed_end_kmh', 'speed_avg_kmh', 'speed_max_kmh', 'speed_min_kmh',
          'rpm_start', 'rpm_end', 'rpm_peak', 'rpm_at_peak_power', 'rpm_at_peak_torque', 'max_throttle_pct',
          'power_peak_corrected', 'torque_peak_nm', 'power_peak_wheel', 'power_peak_crank', 'power_peak_indicated',
          'loss_peak_total', 'loss_peak_aero', 'loss_peak_roll', 'loss_peak_slope',
          'max_slip_pct', 'fuel_avg_lph', 'fuel_max_lph',
          'oil_temp_start_c', 'oil_temp_end_c', 'air_density_kgm3',
          'corr_std', 'corr_factor_k', 'drive_type', 'gearbox_loss_pct',
          'ambient', 'gps', 'vehicle_summary'
        ];
        const wideV = [
          s.modeKey || '',
          s.mode || '',
          s.status || '',
          nStr(s.timeS, 3),
          nStr(s.distanceM, 1),
          nStr(s.startKmh, 2),
          nStr(s.endKmh, 2),
          nStr(s.avgSpeedKmh, 2),
          nStr(s.maxSpeedKmh, 2),
          nStr(s.minSpeedKmh, 2),
          nStr(s.startRpm, 0),
          nStr(s.endRpm, 0),
          nStr(s.peakRpm, 0),
          nStr(s.peakHpCorrRpm, 0),
          nStr(s.peakTorqueRpm, 0),
          nStr(s.maxThrottle, 0),
          nStr(s.peakHp, 2),
          nStr(s.peakTorqueNm, 1),
          nStr(s.peakHpWheel, 2),
          nStr(s.peakHpCrank, 2),
          nStr(s.peakHpIndicated, 2),
          nStr(s.peakLossTotal, 2),
          nStr(s.peakLossAero, 2),
          nStr(s.peakLossRoll, 2),
          nStr(s.peakLossSlope, 2),
          nStr(s.maxSlipPct, 2),
          nStr(s.avgFuelLph, 3),
          nStr(s.maxFuelLph, 3),
          nStr(s.engineOilStartC, 1),
          nStr(s.engineOilEndC, 1),
          nStr(s.airDensity, 5),
          s.corrStd || '',
          nStr(s.corrFactorK, 3),
          s.driveType || '',
          nStr(s.lossGearboxPct, 2),
          s.ambientTxt || '',
          s.gpsTxt || '',
          s.vehicleTxt || ''
        ];
        lines.push(csvLine(['block', 'last_run_summary']));
        lines.push(csvLine(wideH));
        lines.push(csvLine(wideV));
        lines.push('');
        lines.push(csvLine(['block', 'report_print_mirror']));
        lines.push(csvLine(['label', 'value']));
        buildReportPrintMirrorCsvRows(s, u, lastLiveMsg).forEach((pair) => {
          lines.push(csvLine(pair));
        });
        lines.push('');
        if (hasSamples) {
          lines.push(csvLine(['block', 'time_series']));
          lines.push(csvLine(tsHeader));
          sampleRows.forEach((row) => lines.push(row));
        } else {
          lines.push(csvLine(['block', 'time_series']));
          lines.push(csvLine(['note', 'No sample rows (e.g. aborted before enough points, or run buffer cleared). Summary and report mirror still apply.']));
        }
        const out = '\uFEFF' + lines.join('\n');
        const blob = new Blob([out], { type: 'text/csv;charset=utf-8' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        const safeMk = String(s.modeKey || 'run').replace(/[^a-z0-9_-]/gi, '_');
        const isoShort = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        a.download = 'dynotrack_export_' + safeMk + '_' + isoShort + '.csv';
        a.click();
        setTimeout(() => { try { URL.revokeObjectURL(a.href); } catch (e) {} }, 2500);
      }

      let ws = null;
      let reconnectTimer = null;
      let reconnectDelayMs = 200;
      let wsHeartbeatTimer = null;

      /** ARMED (waiting for start): inject plausible GNSS fields so pills / fusion match expectations; real run uses live GPS again. */
      function applyArmedDummyGps(m) {
        if (!m || m.type !== 'live') return m;
        const o = Object.assign({}, m);
        o.gps_lock = true;
        o.gps_sats = Math.max(10, Number(o.gps_sats || 0));
        const hd = Number(o.gps_hdop || 99);
        o.gps_hdop = Math.min(1.35, Math.max(0.85, hd > 20 ? 1.2 : hd));
        o.gnss_mode = 'GPS_3D';
        const la = Number(o.gps_lat || 0);
        const lo = Number(o.gps_lon || 0);
        if (Math.abs(la) < 1e-4 && Math.abs(lo) < 1e-4) {
          o.gps_lat = 41.9981;
          o.gps_lon = 21.4254;
        }
        const sw = Number(o.speed_gps_weight || 0);
        if (sw > 0.5) o.speed_gps_weight = 0.35;
        return o;
      }

      function canAutoArmMeasurement() {
        const m = getMeasurementMode();
        if (!m || m === '__track_nav__') return false;
        if (lastMissingFields.length) return false;
        if (coastCalBlocksControls) return false;
        if (activeScreen !== 'home') return false;
        if (trackPanelVisible) return false;
        return true;
      }

      function modeNeedsCustomApplyFirst(mode) {
        return mode === 'drag_custom' || mode === 'drag_custom_dist' || mode === 'mid_custom' || mode === 'braking_custom';
      }
      function disarmIfWaitingOnly() {
        if (runArmed && !runActive) {
          runArmed = false;
          runReady = false;
          gpsRiskAckForArm = false;
        }
      }
      function modeUsesRoadGps(mode) {
        // Track has its own dedicated lock gate logic.
        // All other modes, including dyno_pull, must carry GNSS risk warnings/consent metadata.
        return !!mode && mode !== '__track_nav__';
      }
      function isGpsWeakForActiveRun(mode, gpsLock, gpsSats, gpsHdop) {
        if (!modeUsesRoadGps(mode)) return false;
        // Active run: only flag clear GNSS drop/degradation, not tiny jitter.
        if (!gpsLock) return true;
        if (Number(gpsSats || 0) < 4) return true;
        if (Number(gpsHdop || 99) > 4.0) return true;
        return false;
      }
      function evaluateGpsMainReadiness(gpsLock, gpsSats, gpsHdop, gnssMode) {
        const lockOk = !!gpsLock && String(gnssMode || 'NONE').toUpperCase() !== 'NO_LOCK';
        const satsOk = Number(gpsSats || 0) >= 10;
        const hdopOk = Number(gpsHdop || 99) <= 1.8;
        return lockOk && satsOk && hdopOk;
      }
      function computeGpsConfidencePct(gpsLock, gpsSats, gpsHdop, gnssMode) {
        const lockScore = (!!gpsLock && String(gnssMode || 'NONE').toUpperCase() !== 'NO_LOCK') ? 100 : 18;
        const sats = Math.max(0, Number(gpsSats || 0));
        const satsScore = Math.max(0, Math.min(100, (sats / 14) * 100));
        const hdop = Math.max(0.6, Number(gpsHdop || 99));
        const hdopScore = Math.max(0, Math.min(100, 100 - ((hdop - 0.7) / 2.5) * 100));
        return Math.max(0, Math.min(100, Math.round(0.45 * lockScore + 0.30 * satsScore + 0.25 * hdopScore)));
      }
      function resetRunGpsTelemetry() {
        runGpsSamples = 0;
        runGpsConfAcc = 0;
        runGpsMinConfPct = 100;
        runGpsMinSats = 99;
        runGpsMaxHdop = 0;
        runGpsHadNoLock = false;
        runStartedWithGpsRisk = false;
      }
      function pushRunGpsTelemetry(gpsLock, gpsSats, gpsHdop, gpsConfidencePct) {
        const sats = Number(gpsSats || 0);
        const hdop = Number(gpsHdop || 99);
        const conf = Number(gpsConfidencePct || 0);
        runGpsSamples += 1;
        runGpsConfAcc += conf;
        if (conf < runGpsMinConfPct) runGpsMinConfPct = conf;
        if (sats < runGpsMinSats) runGpsMinSats = sats;
        if (hdop > runGpsMaxHdop) runGpsMaxHdop = hdop;
        if (!gpsLock) runGpsHadNoLock = true;
      }
      function formatRunGpsTelemetryText() {
        if (runGpsSamples <= 0) return 'run GNSS: no samples';
        const avg = runGpsConfAcc / runGpsSamples;
        return 'run GNSS: avg ' + avg.toFixed(0) + '%'
          + ' | min ' + runGpsMinConfPct.toFixed(0) + '%'
          + ' | sats min ' + runGpsMinSats.toFixed(0)
          + ' | HDOP max ' + runGpsMaxHdop.toFixed(1)
          + ' | lock drop ' + (runGpsHadNoLock ? 'YES' : 'no');
      }
      function runHasGpsWarning() {
        return runStartedWithGpsRisk
          || runGpsDropDetected
          || runGpsHadNoLock
          || runGpsSamples <= 0
          || runGpsMinConfPct < 50
          || runGpsMaxHdop > 4.0;
      }
      function isGpsRiskForCurrentMode(mode) {
        if (!modeUsesRoadGps(mode)) return false;
        return !gpsStatusSnapshot.mainReady;
      }
      function applyCustomModeArmPolicy(modeOpt) {
        const m = modeOpt || getMeasurementMode();
        if (modeNeedsCustomApplyFirst(m) && !customApplySealActive) {
          suppressAutoArm = true;
          disarmIfWaitingOnly();
        } else {
          suppressAutoArm = false;
          if (measurementAutoArmPref) {
            armAutoRunQuiet();
          } else {
            disarmIfWaitingOnly();
            if (m && m !== '__track_nav__' && canAutoArmMeasurement()) {
              allowManualStartRun = true;
            } else {
              allowManualStartRun = false;
            }
          }
        }
      }

      function armAutoRunQuiet() {
        if (runArmed || runActive) return;
        if (!canAutoArmMeasurement()) return;
        if (lastMissingFields.length) {
          if (lastMissingFields.includes('weightKg')) {
            showDtModal('Vehicle weight is required', () => {
              window.location.href = '/settings?focus=weightKg';
            });
          } else {
            showDtModal('Setup incomplete — fill required fields', () => {
              window.location.href = '/settings';
            });
          }
          return;
        }
        const mode = getMeasurementMode();
        if (isGpsRiskForCurrentMode(mode) && !gpsRiskAckForArm) {
          allowManualStartRun = true;
          showGpsRiskAcknowledgeModal(() => {
            gpsRiskAckForArm = true;
            beep(740, 1300);
            armAutoRunQuiet();
            refreshStartRunButton();
          });
          return;
        }
        allowManualStartRun = false;
        runArmed = true;
        runReady = false;
        suppressAutoArm = false;
        sessionPeakPower = 0;
        sessionPeakTorque = 0;
        refreshInteractionLock();
      }

      function wsSetState(state) {
        if (!wsState) return;
        wsState.classList.remove('ws-disconnected', 'ws-connecting', 'ws-connected');
        wsState.classList.add('ws-' + state);
      }

      function scheduleReconnect() {
        if (reconnectTimer) return;
        wsSetState('connecting');
        wsState.textContent = 'OBDII: reconnecting…';
        const jitter = Math.floor(Math.random() * 180);
        const delay = Math.min(5000, reconnectDelayMs + jitter);
        reconnectTimer = setTimeout(() => {
          reconnectTimer = null;
          try {
            connectWs();
          } catch (e) {
            reconnectDelayMs = Math.min(5000, reconnectDelayMs + 100);
            scheduleReconnect();
          }
        }, delay);
        reconnectDelayMs = Math.min(5000, reconnectDelayMs + 100);
      }

      function paintHomeCardsFromMsg(msg) {
        elSpeed.textContent = Number(msg.speed_kmh).toFixed(0);
        elRpm.textContent = Number(msg.rpm).toFixed(0);
        const pNow = powerConvert(Number(msg.hp_corrected));
        const tNow = Number(msg.torque_nm);
        if (pNow > sessionPeakPower) sessionPeakPower = pNow;
        if (tNow > sessionPeakTorque) sessionPeakTorque = tNow;
        elHp.textContent = powerConvert(Number(msg.hp)).toFixed(0);
        elTorque.textContent = Number(msg.torque_nm).toFixed(0);
        elHpWheel.textContent = powerConvert(Number(msg.hp_wheel)).toFixed(0);
        elHpCrank.textContent = powerConvert(Number(msg.hp_crank)).toFixed(0);
        elHpCorr.textContent = powerConvert(Number(msg.hp_corrected)).toFixed(0);
        elThrottle.textContent = Number(msg.throttle_pct).toFixed(0);
        elFuelRate.textContent = Number(msg.fuel_rate_lph).toFixed(1);
      }

      function paintMeasurementGauges(msg) {
        if (!msg || msg.type !== 'live') {
          drawLiveGauges(0, 0, 0, 0);
          return;
        }
        drawLiveGauges(
          Number(msg.speed_kmh),
          Number(msg.rpm),
          powerConvert(Number(msg.hp_corrected)),
          Number(msg.torque_nm)
        );
      }

      function updateBatteryWidget(msg) {
        if (!batteryWidget || !batteryFill) return;
        const pctRaw = Number(msg.battery_pct);
        const state = String(msg.battery_state || 'unknown');
        let prof = String(msg.battery_profile || '');
        if (prof === 'car12') prof = 'external';
        batteryWidget.classList.remove('state-green', 'state-yellow', 'state-red', 'state-unknown', 'state-external');
        const setAllUnknown = () => {
          if (batteryPctVal) batteryPctVal.textContent = '—';
          if (batteryStatusVal) batteryStatusVal.textContent = '—';
        };
        if (prof === 'external') {
          batteryWidget.classList.add('state-external');
          batteryFill.style.width = '100%';
          if (batteryPctVal) batteryPctVal.textContent = 'n/a';
          if (batteryStatusVal) batteryStatusVal.textContent = 'External power';
          return;
        }
        const hasSoc = isFinite(pctRaw) && pctRaw >= 0;
        if (!hasSoc) {
          batteryWidget.classList.add('state-unknown');
          batteryFill.style.width = '8%';
          if (batteryPctVal) batteryPctVal.textContent = '—';
          if (batteryStatusVal) batteryStatusVal.textContent = 'Unknown';
          return;
        }
        const pct = Math.max(0, Math.min(100, pctRaw));
        if (state === 'green') batteryWidget.classList.add('state-green');
        else if (state === 'yellow') batteryWidget.classList.add('state-yellow');
        else if (state === 'red') batteryWidget.classList.add('state-red');
        else batteryWidget.classList.add('state-unknown');
        batteryFill.style.width = pct.toFixed(0) + '%';
        if (batteryPctVal) batteryPctVal.textContent = pct.toFixed(0) + '%';
        let statusTxt = '—';
        if (state === 'green') statusTxt = 'OK';
        else if (state === 'yellow') statusTxt = 'Charge soon';
        else if (state === 'red') statusTxt = 'Low';
        else statusTxt = 'Unknown';
        if (batteryStatusVal) batteryStatusVal.textContent = statusTxt;
      }

      function connectWs() {
        if (wsHeartbeatTimer) {
          try { clearInterval(wsHeartbeatTimer); } catch (e) {}
          wsHeartbeatTimer = null;
        }
        if (ws) {
          try {
            ws.onopen = null;
            ws.onclose = null;
            ws.onerror = null;
            ws.onmessage = null;
            ws.close();
          } catch (e) {}
          ws = null;
        }
        wsSetState('connecting');
        wsState.textContent = 'OBDII: connecting…';
        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
          reconnectDelayMs = 200;
          // Must reset: old timestamp + open socket made "stall" logic think the link was dead
          // immediately and forced close() in a loop, blocking stable reconnect.
          lastWsMsgAt = 0;
          lastSampleTms = 0;
          if (elLastLiveInfo) elLastLiveInfo.textContent = 'Last live update: —';
          wsSetState('connected');
          wsState.textContent = 'OBDII connected';
          if (wsHeartbeatTimer) {
            try { clearInterval(wsHeartbeatTimer); } catch (e) {}
            wsHeartbeatTimer = null;
          }
          wsHeartbeatTimer = setInterval(() => {
            try {
              if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('{"type":"hb"}');
              }
            } catch (e) {}
          }, 10000);
        };
        ws.onclose = () => {
          if (wsHeartbeatTimer) {
            try { clearInterval(wsHeartbeatTimer); } catch (e) {}
            wsHeartbeatTimer = null;
          }
          wsSetState('connecting');
          wsState.textContent = 'OBDII: reconnecting…';
          scheduleReconnect();
        };
        ws.onerror = () => {
          wsSetState('disconnected');
          wsState.textContent = 'OBDII: error';
          // onclose usually follows; if not, scheduleReconnect is idempotent (single timer).
          scheduleReconnect();
        };
        ws.onmessage = async (event) => {
          let raw = event.data;
          if (raw instanceof Blob) raw = await raw.text();
          else if (raw instanceof ArrayBuffer) raw = new TextDecoder().decode(new Uint8Array(raw));
          else if (raw && raw.buffer instanceof ArrayBuffer && !(typeof raw === 'string')) raw = new TextDecoder().decode(raw);
          let msg;
          try {
            msg = JSON.parse(String(raw));
          } catch (e) {
            return;
          }
          if (!msg || msg.type !== 'live') return;
          if (runArmed && !runActive && getMeasurementMode() !== '__track_nav__') msg = applyArmedDummyGps(msg);
          lastWsMsgAt = Date.now();
          lastLiveMsg = msg;
          if (typeof msg.auto_arm_kmh === 'number' && isFinite(msg.auto_arm_kmh)) {
            cachedAutoArmKmh = Math.max(0, Math.min(200, msg.auto_arm_kmh));
          }
          if (typeof msg.measurement_auto_arm === 'boolean') {
            measurementAutoArmPref = msg.measurement_auto_arm;
          }
          updateSetup(msg);
          updateCoastCal(msg);
          if (coastCalInProgress) {
            const speed = Number(msg.speed_kmh);
            if (speed >= 100) {
              // Removed beep: beep(1000, 200);
            } else if (speed <= 80) {
              coastCalInProgress = false;
              calMsg.textContent = 'Calibration Done.';
              setTimeout(() => {
                if (calMsg) calMsg.textContent = 'Repeat coast-down calibration procedure.';
              }, 3000);
              // Removed beep: beep(1500, 300);
            }
          }
          if (msg.power_unit && (msg.power_unit === 'hp' || msg.power_unit === 'kw') && powerUnit !== msg.power_unit) {
            powerUnit = msg.power_unit;
            refreshPowerUnitLabels();
          }
          // Gauge paint runs AFTER run state machine (below) so runActive matches this frame.
          if (elLastLiveInfo) elLastLiveInfo.textContent = 'Last live update: just now';
          const iatC = Number(msg.air_intake_c);
          const oilC = Number(msg.engine_oil_c);
          elIat.textContent = iatC.toFixed(0);
          elOilTemp.textContent = oilC.toFixed(0);
          elLossInfo.textContent = 'Loss total: ' + powerConvert(Number(msg.hp_loss_total)).toFixed(1)
            + ' ' + powerUnitLabel();
          elCorrInfo.textContent = 'Corr: ' + String(msg.corr_std || 'DIN').toUpperCase()
            + ' x' + Number(msg.corr_factor_k).toFixed(3);
          elAnomalyInfo.textContent = 'Anomaly: ' + String(msg.anomaly || 'none');
          elSelfCalInfo.textContent = 'SelfCal: ' + ((msg.self_cal_enabled) ? 'ON' : 'OFF')
            + (msg.self_cal_locked ? ' LOCK' : '') + ' (' + Number(msg.self_cal_conf || 0).toFixed(0) + '%)';
          const gpsLock = !!msg.gps_lock;
          const gpsSats = Number(msg.gps_sats || 0);
          const gpsHdop = Number(msg.gps_hdop || 99);
          gpsLat = Number(msg.gps_lat || 0);
          gpsLon = Number(msg.gps_lon || 0);
          redlineRpm = Number(msg.redline_rpm || 6500);
          const gnssMode = String(msg.gnss_mode || 'NONE').toUpperCase();
          const gpsMainReady = evaluateGpsMainReadiness(gpsLock, gpsSats, gpsHdop, gnssMode);
          const gpsConfidencePct = computeGpsConfidencePct(gpsLock, gpsSats, gpsHdop, gnssMode);
          gpsStatusSnapshot.lock = gpsLock;
          gpsStatusSnapshot.sats = gpsSats;
          gpsStatusSnapshot.hdop = gpsHdop;
          gpsStatusSnapshot.gnssMode = gnssMode;
          gpsStatusSnapshot.mainReady = gpsMainReady;
          gpsStatusSnapshot.confidencePct = gpsConfidencePct;
          elGpsLockInfo.textContent = gpsLock
            ? ('GPS: LOCK ' + gnssMode + ' | sats ' + gpsSats + ' | HDOP ' + gpsHdop.toFixed(1) + ' | conf ' + gpsConfidencePct + '%')
            : ('GPS: NO LOCK | sats ' + gpsSats + ' | HDOP ' + gpsHdop.toFixed(1) + ' | conf ' + gpsConfidencePct + '% -> Move device on dashboard under windshield, antenna facing sky.');
          elGpsLockInfo.classList.remove('health-normal', 'health-warn', 'health-hot');
          if (gpsLock && gpsHdop <= 1.6 && gpsSats >= 10) elGpsLockInfo.classList.add('health-normal');
          else if (gpsLock && gpsHdop <= 2.5 && gpsSats >= 7) elGpsLockInfo.classList.add('health-warn');
          else elGpsLockInfo.classList.add('health-hot');
          if (elGpsConstellationInfo) {
            elGpsConstellationInfo.textContent = gpsMainReady
              ? ('GNSS ready (' + gpsConfidencePct + '%): main lock is stable. Up to 4 constellations can be used at once (GPS+GLONASS+Galileo+BeiDou), with many satellites tracked.')
              : ('GNSS caution (' + gpsConfidencePct + '%): waiting stable main lock. Target is 4 constellations (GPS+GLONASS+Galileo+BeiDou); satellite count can be much higher than 4.');
            elGpsConstellationInfo.classList.remove('health-normal', 'health-warn', 'health-hot');
            if (gpsConfidencePct >= 78 && gpsMainReady) elGpsConstellationInfo.classList.add('health-normal');
            else if (gpsConfidencePct >= 50) elGpsConstellationInfo.classList.add('health-warn');
            else elGpsConstellationInfo.classList.add('health-hot');
          }
          if (elGpsConfidenceBadge) {
            elGpsConfidenceBadge.classList.remove('health-normal', 'health-warn', 'health-hot');
            if (gpsConfidencePct >= 78 && gpsMainReady) {
              elGpsConfidenceBadge.classList.add('health-normal');
              elGpsConfidenceBadge.textContent = 'GNSS confidence: ' + gpsConfidencePct + '% (READY) - primary lock stable for measurement.';
            } else if (gpsConfidencePct >= 50) {
              elGpsConfidenceBadge.classList.add('health-warn');
              elGpsConfidenceBadge.textContent = 'GNSS confidence: ' + gpsConfidencePct + '% (CAUTION) - measurement possible, but wait for better lock for best accuracy.';
            } else {
              elGpsConfidenceBadge.classList.add('health-hot');
              elGpsConfidenceBadge.textContent = 'GNSS confidence: ' + gpsConfidencePct + '% (LOW) - no stable lock; results may be less accurate.';
            }
            const gnssTier = gnssBadgeLevelFromReadiness(gpsConfidencePct, gpsMainReady);
            const mmSound = getMeasurementMode();
            const roadRunMeasuring = runActive && modeUsesRoadGps(mmSound);
            const trackRunMeasuring = runActive && mmSound === '__track_nav__';
            updateGnssBadgeAlertSounds(gnssBadgeAlertPrevLevel, gnssTier, {
              silenceBadgeDuringRoadRun: roadRunMeasuring,
              suppressYellowDuringRun: trackRunMeasuring
            });
          }
          elApClientsInfo.textContent = 'AP clients: ' + Number(msg.ap_clients || 0).toFixed(0);
          elNoiseInfo.textContent = 'Noise: ' + Number(msg.signal_noise_pct || 0).toFixed(1) + '%';
          elDriftInfo.textContent = 'Drift: ' + Number(msg.signal_drift_pct || 0).toFixed(1) + '%';
          elResInfo.textContent = 'Resolution: ' + Number(msg.signal_resolution_pct || 100).toFixed(1) + '%';
          const gpsW = Number(msg.speed_gps_weight || 0);
          const obdW = 1 - gpsW;
          const obdKmh = Number(msg.speed_obd_kmh || 0);
          elFusionInfo.textContent = 'Fusion: OBD ' + (obdW * 100).toFixed(0) + '% / GPS ' + (gpsW * 100).toFixed(0) + '%';
          // Road modes: while ARMED (waiting for start), never block on GNSS — real GPS applies again once run is active.
          const gpsOkForAccelModes = (runArmed && !runActive)
            || (gpsLock && gpsSats >= 4 && gpsHdop <= 4.0)
            || (gpsW < 0.55)
            || (obdKmh > 3 && gpsW < 0.75);
          elSlopeInfo.textContent = 'Auto slope: ' + Number(msg.auto_slope_pct || 0).toFixed(2) + '%';
          elWindInfo.textContent = 'Wind est: ' + Number(msg.wind_kmh || 0).toFixed(1) + ' km/h';
          const gInt = Number(msg.gear_detected_int || 0);
          const gFloat = Number(msg.gear_detected || 0);
          elGearInfo.textContent = 'Gear: ' + (gInt > 0 ? String(gInt) : '-') + ' (' + gFloat.toFixed(2) + ')';
          elSlipInfo.textContent = 'Slip: ' + Number(msg.slip_pct || 0).toFixed(1) + '%';
          elSlipInfo.classList.remove('health-normal', 'health-warn', 'health-hot');
          const slip = Number(msg.slip_pct || 0);
          if (slip < 4) elSlipInfo.classList.add('health-normal');
          else if (slip < 9) elSlipInfo.classList.add('health-warn');
          else elSlipInfo.classList.add('health-hot');
          const noise = Number(msg.signal_noise_pct || 0);
          const drift = Number(msg.signal_drift_pct || 0);
          const res = Number(msg.signal_resolution_pct || 100);
          const qScore = Math.max(0, Math.min(100, 0.45 * (100 - noise) + 0.35 * (100 - drift) + 0.20 * res));
          const qLabel = qScore >= 80 ? 'Excellent' : (qScore >= 60 ? 'Good' : 'Poor');
          elQualityInfo.textContent = 'Quality: ' + qLabel + ' (' + qScore.toFixed(0) + '%)';
          elQualityInfo.classList.remove('health-normal', 'health-warn', 'health-hot');
          if (qScore >= 80) elQualityInfo.classList.add('health-normal');
          else if (qScore >= 60) elQualityInfo.classList.add('health-warn');
          else elQualityInfo.classList.add('health-hot');
          setHealthVisual(classifyIat(iatC), elIatHealth, iatCard, 'IAT');
          setHealthVisual(classifyOil(oilC), elOilHealth, oilCard, 'Oil');
          elTInfo.textContent = 't_ms: ' + msg.t_ms;

          const sample = {
            t_ms: Number(msg.t_ms), rpm: Number(msg.rpm), speed_kmh: Number(msg.speed_kmh),
            hp: Number(msg.hp),
            hp_wheel: Number(msg.hp_wheel), hp_crank: Number(msg.hp_crank), hp_corrected: Number(msg.hp_corrected),
            hp_loss_total: Number(msg.hp_loss_total),
            hp_loss_aero: Number(msg.hp_loss_aero || 0),
            hp_loss_roll: Number(msg.hp_loss_roll || 0),
            hp_loss_slope: Number(msg.hp_loss_slope || 0),
            torque_nm: Number(msg.torque_nm), throttle_pct: Number(msg.throttle_pct), fuel_rate_lph: Number(msg.fuel_rate_lph),
            air_intake_c: Number(msg.air_intake_c), engine_oil_c: Number(msg.engine_oil_c), anomaly: String(msg.anomaly || ''),
            slip_pct: Number(msg.slip_pct || 0),
            air_density: Number(msg.air_density || 0)
          };
          if (trackSessionActive && gpsLock && gpsLat !== 0 && gpsLon !== 0) {
            if (trackStartRunLapArm && trackGateLat != null && trackGateLon != null) {
              const distToGate = haversineMeters(gpsLat, gpsLon, trackGateLat, trackGateLon);
              const inGateZone = distToGate <= TRACK_ZONE_M;
              if (runArmed && !runActive && trackAwaitingSpeedForLap) {
                if (sample.speed_kmh >= cachedAutoArmKmh) {
                  runActive = true;
                  runGpsDropDetected = false;
                  runGpsDropNotified = false;
                  lastRunHadGpsDrop = false;
                  lastGpsWeakRoadAlertMs = 0;
                  resetRunGpsTelemetry();
                  runStartedWithGpsRisk = !!gpsRiskAckForArm || isGpsRiskForCurrentMode('__track_nav__');
                  pushRunGpsTelemetry(gpsLock, gpsSats, gpsHdop, gpsConfidencePct);
                  trackAwaitingSpeedForLap = false;
                  trackLapStartTms = sample.t_ms;
                  trackRunHasLeftGate = false;
                  trackCurrentLapNo = trackLaps.length + 1;
                  if (trackCurrentLapNo < 1) trackCurrentLapNo = 1;
                  resetTrackCurrentLapStats();
                  currentRun = [];
                  runDistanceM = 0;
                  runDistancePrevM = 0;
                  runStartTms = sample.t_ms;
                  runDisplayStartPerf = performance.now();
                  runStartSpeedKmh = sample.speed_kmh;
                  runLastSpeedKmh = sample.speed_kmh;
                  runLastSpeedChangeTms = sample.t_ms;
                  prevRunSample = sample;
                  dynoPeakRpm = sample.rpm;
                  drawCurve([]);
                  showRunCue('go', 1400);
                  refreshInteractionLock();
                  refreshStartRunButton();
                  scrollHomeToAnalogGauges();
                  vibrate([70, 40, 70]);
                  flash('rgba(80,180,255,0.55)', 170);
                  // Beep now in showRunCue
                }
              }
              if (runActive && getMeasurementMode() === '__track_nav__') {
                trackCurrentTopSpeed = Math.max(trackCurrentTopSpeed, sample.speed_kmh);
                trackCurrentSpeedSum += sample.speed_kmh;
                trackCurrentSamples += 1;
                if (trackCurrentRpmTrend.length > 700) trackCurrentRpmTrend.shift();
                if (trackCurrentSpeedTrend.length > 700) trackCurrentSpeedTrend.shift();
                if (trackCurrentHpTrend.length > 700) trackCurrentHpTrend.shift();
                trackCurrentRpmTrend.push(sample.rpm);
                trackCurrentSpeedTrend.push(sample.speed_kmh);
                trackCurrentHpTrend.push(sample.hp_corrected);
                if (distToGate >= TRACK_MIN_LEAVE_M) trackRunHasLeftGate = true;
                const lapElapsedMs = sample.t_ms - trackLapStartTms;
                const stoppedAtGate = inGateZone && sample.speed_kmh <= TRACK_STANDSTILL_KMH;
                if (trackRunHasLeftGate && stoppedAtGate && lapElapsedMs >= TRACK_MIN_LAP_MS) {
                  const lapTimeS = lapElapsedMs / 1000.0;
                  const avgSpeed = trackCurrentSamples > 0 ? (trackCurrentSpeedSum / trackCurrentSamples) : 0;
                  const lap = {
                    lap_number: trackLaps.length + 1,
                    time: lapTimeS,
                    best_lap: false,
                    top_speed: trackCurrentTopSpeed,
                    avg_speed: avgSpeed,
                    rpm_trend: trackCurrentRpmTrend.slice(),
                    speed_trend: trackCurrentSpeedTrend.slice(),
                    hp_trend: trackCurrentHpTrend.slice()
                  };
                  trackLaps.push(lap);
                  if (trackBestLapS <= 0 || lapTimeS < trackBestLapS) {
                    trackBestLapS = lapTimeS;
                    trackLaps.forEach(x => { x.best_lap = false; });
                    lap.best_lap = true;
                  }
                  trackCurrentLapNo = lap.lap_number + 1;
                  runActive = false;
                  runDisplayStartPerf = 0;
                  trackAwaitingSpeedForLap = true;
                  trackRunHasLeftGate = false;
                  resetTrackCurrentLapStats();
                  currentRun = [];
                  runDistanceM = 0;
                  renderTrackLaps();
                  updateTrackHeader();
                  syncTrackSessionToMeasurementSummary();
                  refreshInteractionLock();
                  refreshStartRunButton();
                  lastMainResult = (() => {
                    const min = Math.floor(lapTimeS / 60);
                    const sec = (lapTimeS % 60).toFixed(2);
                    return min > 0 ? min + 'min ' + sec + 'sec' : sec + ' sec';
                  })();
                  showRunCue('finish', 2200);
                  vibrate([160, 80, 160]);
                  flash('rgba(0,255,140,0.55)', 260);
                }
              }
            } else if (!trackStartRunLapArm && trackOriginLat === null && trackOriginLon === null && sample.speed_kmh <= TRACK_STANDSTILL_KMH) {
              trackOriginLat = gpsLat;
              trackOriginLon = gpsLon;
              trackStartLat = gpsLat;
              trackStartLon = gpsLon;
              trackLapStartTms = sample.t_ms;
              trackMaxDistFromOriginM = 0;
              trackHasLeftOrigin = false;
              trackLapInZone = true;
              trackCurrentLapNo = trackLaps.length + 1;
              if (trackCurrentLapNo < 1) trackCurrentLapNo = 1;
              resetTrackCurrentLapStats();
              updateTrackHeader();
            }
            if (!trackStartRunLapArm && trackOriginLat !== null && trackOriginLon !== null) {
              const distToOrigin = haversineMeters(gpsLat, gpsLon, trackOriginLat, trackOriginLon);
              trackMaxDistFromOriginM = Math.max(trackMaxDistFromOriginM, distToOrigin);
              if (trackMaxDistFromOriginM >= TRACK_MIN_LEAVE_M) trackHasLeftOrigin = true;

              trackCurrentTopSpeed = Math.max(trackCurrentTopSpeed, sample.speed_kmh);
              trackCurrentSpeedSum += sample.speed_kmh;
              trackCurrentSamples += 1;
              if (trackCurrentRpmTrend.length > 700) trackCurrentRpmTrend.shift();
              if (trackCurrentSpeedTrend.length > 700) trackCurrentSpeedTrend.shift();
              if (trackCurrentHpTrend.length > 700) trackCurrentHpTrend.shift();
              trackCurrentRpmTrend.push(sample.rpm);
              trackCurrentSpeedTrend.push(sample.speed_kmh);
              trackCurrentHpTrend.push(sample.hp_corrected);

              const inZone = distToOrigin <= TRACK_ZONE_M;
              const lapElapsedMs = sample.t_ms - trackLapStartTms;
              if (trackHasLeftOrigin && inZone && !trackLapInZone && lapElapsedMs >= TRACK_MIN_LAP_MS) {
                const lapTimeS = lapElapsedMs / 1000.0;
                const avgSpeed = trackCurrentSamples > 0 ? (trackCurrentSpeedSum / trackCurrentSamples) : 0;
                const lap = {
                  lap_number: trackLaps.length + 1,
                  time: lapTimeS,
                  best_lap: false,
                  top_speed: trackCurrentTopSpeed,
                  avg_speed: avgSpeed,
                  rpm_trend: trackCurrentRpmTrend.slice(),
                  speed_trend: trackCurrentSpeedTrend.slice(),
                  hp_trend: trackCurrentHpTrend.slice()
                };
                trackLaps.push(lap);
                if (trackBestLapS <= 0 || lapTimeS < trackBestLapS) {
                  trackBestLapS = lapTimeS;
                  trackLaps.forEach(x => { x.best_lap = false; });
                  lap.best_lap = true;
                }
                trackCurrentLapNo = lap.lap_number + 1;
                trackLapStartTms = sample.t_ms;
                trackMaxDistFromOriginM = 0;
                trackHasLeftOrigin = false;
                resetTrackCurrentLapStats();
                renderTrackLaps();
                updateTrackHeader();
                syncTrackSessionToMeasurementSummary();
                showRunCue('finish', 2200);
                vibrate([160, 80, 160]);
                flash('rgba(0,255,140,0.55)', 260);
              }
              trackLapInZone = inZone;
            }
          }
          if (trackSessionActive) updateTrackSmartHint();
          const mode = getMeasurementMode();
          const cr = parseCustomRange();
          const cd = parseCustomDragDistM();
          const presetD = dragStripPresetM(mode);
          const dragTargetM = (isFinite(presetD) && presetD > 0) ? presetD
            : (mode === 'drag_custom_dist' && cd.valid ? cd.meters : -1);
          let dtMs = 0;
          if (lastSampleTms > 0 && sample.t_ms >= lastSampleTms) {
            dtMs = sample.t_ms - lastSampleTms;
            if (dtMs > 750) dtMs = 750;
          }
          lastSampleTms = sample.t_ms;
          if (runActive && dtMs > 0) {
            runDistancePrevM = runDistanceM;
            const prevV = prevRunSample ? (prevRunSample.speed_kmh / 3.6) : (sample.speed_kmh / 3.6);
            const curV = sample.speed_kmh / 3.6;
            runDistanceM += ((prevV + curV) * 0.5) * (dtMs / 1000.0);
          }

          if (suppressAutoArm && sample.speed_kmh < 8) suppressAutoArm = false;
          if (measurementAutoArmPref && !runArmed && !runActive && !suppressAutoArm && canAutoArmMeasurement()) {
            const thr = cachedAutoArmKmh;
            if (sample.speed_kmh >= thr) armAutoRunQuiet();
          }

          if (runArmed && !runActive && needsStandstillWarmup(mode)) {
            if (sample.speed_kmh < 3) runReady = true;
          }

          let startTrigger = false;
          if (runArmed && !runActive && mode !== '__track_nav__' && (mode === 'dyno_pull' || gpsOkForAccelModes)) {
            switch (mode) {
              case 'drag_0_100':
                startTrigger = (runReady && sample.speed_kmh >= 5)
                  || (sample.speed_kmh >= 5 && sample.speed_kmh <= 36);
                break;
              case 'drag_0_200':
                startTrigger = (runReady && sample.speed_kmh >= 5)
                  || (sample.speed_kmh >= 5 && sample.speed_kmh <= 42);
                break;
              case 'mid_100_200':
                startTrigger = sample.speed_kmh >= 100 && sample.throttle_pct > 25;
                break;
              case 'drag_201m':
              case 'drag_402m':
              case 'drag_804m':
                startTrigger = sample.speed_kmh >= 5;
                break;
              case 'drag_custom_dist':
                startTrigger = cd.valid && sample.speed_kmh >= 5;
                break;
              case 'drag_custom':
                if (!cr.valid) startTrigger = false;
                else if (cr.lo < 15) {
                  const rollMaxKmh = cr.hi <= 100 ? 36 : 42;
                  startTrigger = (runReady && sample.speed_kmh >= 5)
                    || (sample.speed_kmh >= 5 && sample.speed_kmh <= rollMaxKmh);
                } else startTrigger = sample.speed_kmh >= cr.lo && sample.throttle_pct > 25;
                break;
              case 'mid_60_100':
                startTrigger = sample.speed_kmh >= 60 && sample.throttle_pct > 25;
                break;
              case 'mid_80_120':
                startTrigger = sample.speed_kmh >= 80 && sample.throttle_pct > 25;
                break;
              case 'mid_custom':
                startTrigger = cr.valid && sample.speed_kmh >= cr.lo && sample.throttle_pct > 25;
                break;
              case 'braking_100_0':
                startTrigger = sample.speed_kmh >= 100;
                break;
              case 'braking_custom': {
                const br = parseBrakingRange();
                startTrigger = br.valid && sample.speed_kmh >= br.from;
                break;
              }
              case 'dyno_pull':
                startTrigger = sample.rpm >= 1800 && sample.throttle_pct > 40;
                break;
              default:
                break;
            }
          }

          if (startTrigger) {
            runActive = true;
            runGpsDropDetected = false;
            runGpsDropNotified = false;
            lastRunHadGpsDrop = false;
            lastGpsWeakRoadAlertMs = 0;
            resetRunGpsTelemetry();
            runStartedWithGpsRisk = !!gpsRiskAckForArm || isGpsRiskForCurrentMode(mode);
            pushRunGpsTelemetry(gpsLock, gpsSats, gpsHdop, gpsConfidencePct);
            currentRun = [];
            runDistanceM = 0;
            runDistancePrevM = 0;
            runStartSpeedKmh = sample.speed_kmh;
            runStartTms = sample.t_ms;
            runDisplayStartPerf = performance.now();
            runLastSpeedKmh = sample.speed_kmh;
            runLastSpeedChangeTms = sample.t_ms;
            prevRunSample = sample;
            runReady = false;
            dynoPeakRpm = sample.rpm;
            if (mode !== 'dyno_pull') drawCurve([]);
            showRunCue('go', 1400);
            refreshInteractionLock();
            scrollHomeToAnalogGauges();
            vibrate([70, 40, 70]);
            flash('rgba(80,180,255,0.55)', 170);
            // Beep now in showRunCue
          }

          let stopTrigger = false;
          if (runActive) {
            pushRunGpsTelemetry(gpsLock, gpsSats, gpsHdop, gpsConfidencePct);
          }
          if (runActive && isGpsWeakForActiveRun(mode, gpsLock, gpsSats, gpsHdop)) {
            runGpsDropDetected = true;
            lastRunHadGpsDrop = true;
            const nowWeak = Date.now();
            if (nowWeak - lastGpsWeakRoadAlertMs >= GPS_WEAK_ROAD_ALERT_COOLDOWN_MS) {
              lastGpsWeakRoadAlertMs = nowWeak;
              runGpsDropNotified = true;
              elAutoRunReasonInfo.textContent = 'Reason: GNSS degraded during run - result may be less accurate.';
              playGnssRedCritical();
              lastGnssRedCriticalPlayMs = nowWeak;
              flash('rgba(188,18,18,0.48)', 320);
            }
          }
          if (runActive) {
            switch (mode) {
              case 'drag_0_100':
                stopTrigger = Number(sample.speed_kmh) >= 99.0;
                break;
              case 'drag_0_200':
                stopTrigger = Number(sample.speed_kmh) >= 199.0;
                break;
              case 'mid_100_200':
                stopTrigger = Math.round(Number(sample.speed_kmh)) >= 200;
                break;
              case 'drag_201m':
              case 'drag_402m':
              case 'drag_804m':
              case 'drag_custom_dist':
                stopTrigger = dragTargetM > 0 && runDistanceM >= dragTargetM;
                break;
              case 'drag_custom':
                stopTrigger = cr.valid && Number(sample.speed_kmh) >= (cr.hi - 0.5);
                break;
              case 'mid_60_100':
                stopTrigger = Math.round(Number(sample.speed_kmh)) >= 100;
                break;
              case 'mid_80_120':
                stopTrigger = Math.round(Number(sample.speed_kmh)) >= 120;
                break;
              case 'mid_custom':
                stopTrigger = cr.valid && Number(sample.speed_kmh) >= (cr.hi - 0.5);
                break;
              case 'braking_100_0':
                stopTrigger = sample.speed_kmh <= 1;
                break;
              case 'braking_custom': {
                const br = parseBrakingRange();
                stopTrigger = br.valid && sample.speed_kmh <= br.to;
                break;
              }
              case 'dyno_pull':
                dynoPeakRpm = Math.max(dynoPeakRpm, sample.rpm);
                stopTrigger = (sample.rpm > 6400) || (dynoPeakRpm > 3500 && sample.rpm < dynoPeakRpm - 350);
                break;
              default:
                break;
            }
          }
          if (runActive) {
            msPowerWheel.textContent = powerConvert(Number(sample.hp_wheel)).toFixed(0);
            lastAccelMps2 = Number(msg.accel_mps2 || 0);
            msG.textContent = (lastAccelMps2 / 9.81).toFixed(2);
            currentRun.push(sample);
            if (currentRun.length > maxRunPoints) currentRun.shift();
            if (mode === 'dyno_pull') {
              const nowPaint = performance.now();
              if (nowPaint - lastDynoDrawAtMs >= 180) {
                lastDynoDrawAtMs = nowPaint;
                // Downsample for performance (preserve shape without drawing 2000+ points).
                const maxPlotPoints = 450;
                let plotPoints = currentRun;
                if (currentRun.length > maxPlotPoints) {
                  const step = Math.ceil(currentRun.length / maxPlotPoints);
                  plotPoints = [];
                  for (let i = 0; i < currentRun.length; i += step) plotPoints.push(currentRun[i]);
                  if (plotPoints.length < 2) plotPoints = currentRun.slice(-2);
                }
                drawCurve(plotPoints);
              }
            }

            const nowRpt = performance.now();
            if (nowRpt - lastReportUpdateAtMs >= 250) {
              lastReportUpdateAtMs = nowRpt;
              updateReportFromPoints(currentRun, msg);
            }

            const speedDelta = Math.abs(sample.speed_kmh - runLastSpeedKmh);
            if (speedDelta >= 1.0) {
              runLastSpeedKmh = sample.speed_kmh;
              runLastSpeedChangeTms = sample.t_ms;
            }
            const elapsedS = runDisplayStartPerf > 0
              ? Math.max(0, (performance.now() - runDisplayStartPerf) / 1000)
              : Math.max(0, (sample.t_ms - runStartTms) / 1000.0);
            msRunTime.textContent = elapsedS.toFixed(2);
            msRunDistance.textContent = runDistanceM.toFixed(1);
            const noSpeedChangeS = (sample.t_ms - runLastSpeedChangeTms) / 1000.0;
            let nearSpeedTarget = false;
            let timeoutS = 20.0;
            if (mode === '__track_nav__') {
              nearSpeedTarget = true;
              timeoutS = 3600.0;
            } else if (mode === 'drag_0_100' && sample.speed_kmh >= 88) nearSpeedTarget = true;
            else if (mode === 'drag_0_200' && sample.speed_kmh >= 185) nearSpeedTarget = true;
            else if (mode === 'mid_60_100' && sample.speed_kmh >= 92) nearSpeedTarget = true;
            else if (mode === 'mid_80_120' && sample.speed_kmh >= 112) nearSpeedTarget = true;
            else if (mode === 'mid_100_200' && sample.speed_kmh >= 190) nearSpeedTarget = true;
            else if (mode === 'mid_custom' && cr.valid && sample.speed_kmh >= cr.hi - 8) nearSpeedTarget = true;
            else if (mode === 'drag_custom' && cr.valid && sample.speed_kmh >= cr.hi - 8) nearSpeedTarget = true;
            else if (dragTargetM > 0 && runDistanceM >= dragTargetM - 45) nearSpeedTarget = true;
            if (dragTargetM > 0) timeoutS = Math.max(40.0, dragTargetM / 10.0);
            else if (mode === 'drag_0_200') timeoutS = 60.0;
            else if (mode === 'drag_custom' && cr.hi > 150) timeoutS = 60.0;
            else if (mode === 'dyno_pull') timeoutS = 120.0;
            if (elapsedS > timeoutS) {
              abortRun('Timeout exceeded (' + timeoutS.toFixed(0) + 's)');
            } else if (mode !== 'dyno_pull' && !nearSpeedTarget && noSpeedChangeS > 6.0) {
              abortRun('No speed change detected');
            }
          } else {
            if (activeScreen === 'home') {
              msPowerWheel.textContent = powerConvert(Number(sample.hp_wheel)).toFixed(0);
              lastAccelMps2 = Number(msg.accel_mps2 || 0);
              msG.textContent = (lastAccelMps2 / 9.81).toFixed(2);
            } else {
              msPowerWheel.textContent = '0';
              msG.textContent = '0.00';
            }
            msRunTime.textContent = '0.00';
            msRunDistance.textContent = '0.0';
          }
          if (runActive && dragTargetM > 0 && stopTrigger && prevRunSample) {
            const span = runDistanceM - runDistancePrevM;
            if (span > 0.0001) {
              const frac = (dragTargetM - runDistancePrevM) / span;
              const f = Math.max(0, Math.min(1, frac));
              const interp = {
                t_ms: prevRunSample.t_ms + f * (sample.t_ms - prevRunSample.t_ms),
                rpm: prevRunSample.rpm + f * (sample.rpm - prevRunSample.rpm),
                speed_kmh: prevRunSample.speed_kmh + f * (sample.speed_kmh - prevRunSample.speed_kmh),
                hp: Number(prevRunSample.hp || 0) + f * (sample.hp - Number(prevRunSample.hp || 0)),
                hp_wheel: Number(prevRunSample.hp_wheel || 0) + f * (sample.hp_wheel - Number(prevRunSample.hp_wheel || 0)),
                hp_crank: Number(prevRunSample.hp_crank || 0) + f * (sample.hp_crank - Number(prevRunSample.hp_crank || 0)),
                hp_corrected: Number(prevRunSample.hp_corrected || 0) + f * (sample.hp_corrected - Number(prevRunSample.hp_corrected || 0)),
                hp_loss_total: Number(prevRunSample.hp_loss_total || 0) + f * (sample.hp_loss_total - Number(prevRunSample.hp_loss_total || 0)),
                hp_loss_aero: Number(prevRunSample.hp_loss_aero || 0) + f * (sample.hp_loss_aero - Number(prevRunSample.hp_loss_aero || 0)),
                hp_loss_roll: Number(prevRunSample.hp_loss_roll || 0) + f * (sample.hp_loss_roll - Number(prevRunSample.hp_loss_roll || 0)),
                hp_loss_slope: Number(prevRunSample.hp_loss_slope || 0) + f * (sample.hp_loss_slope - Number(prevRunSample.hp_loss_slope || 0)),
                torque_nm: prevRunSample.torque_nm + f * (sample.torque_nm - prevRunSample.torque_nm),
                throttle_pct: prevRunSample.throttle_pct + f * (sample.throttle_pct - prevRunSample.throttle_pct),
                fuel_rate_lph: prevRunSample.fuel_rate_lph + f * (sample.fuel_rate_lph - prevRunSample.fuel_rate_lph),
                air_intake_c: prevRunSample.air_intake_c + f * (sample.air_intake_c - prevRunSample.air_intake_c),
                engine_oil_c: prevRunSample.engine_oil_c + f * (sample.engine_oil_c - prevRunSample.engine_oil_c),
                slip_pct: Number(prevRunSample.slip_pct || 0) + f * (sample.slip_pct - Number(prevRunSample.slip_pct || 0)),
                air_density: Number(prevRunSample.air_density || 0) + f * (sample.air_density - Number(prevRunSample.air_density || 0)),
                anomaly: sample.anomaly
              };
              currentRun[currentRun.length - 1] = interp;
              runDistanceM = dragTargetM;
            }
          }
          const runElapsedMs = sample.t_ms - runStartTms;
          const minRunMsToComplete = (mode === 'dyno_pull') ? 350 : 50;
          if (stopTrigger && currentRun.length >= 2
            && (runElapsedMs >= minRunMsToComplete || currentRun.length >= 6)) {
            lastRunElapsedS = Math.max(0, (sample.t_ms - runStartTms) / 1000.0);
            lastRunDistanceM = runDistanceM;
            msRunTime.textContent = lastRunElapsedS.toFixed(2);
            msRunDistance.textContent = lastRunDistanceM.toFixed(1);
            const hadRunGpsWarning = runHasGpsWarning();
            const resultStatus = hadRunGpsWarning ? 'completed - GNSS warning' : 'completed';
            setMeasurementResultFromRun(mode, currentRun, runDistanceM, resultStatus, lastLiveMsg);
            if (mode === 'dyno_pull' && currentRun.length) {
              updateReportFromPoints(currentRun, lastLiveMsg || {});
            }
            if (hadRunGpsWarning) {
              showDtModal('GNSS signal dropped/degraded during this run. Measurement finished, but precision may be reduced. For best accuracy, repeat the run after stable GPS lock (device under windshield, clear sky view).');
            }
            showRunCue('finish', 2200);
            runActive = false;
            runArmed = false;
            gpsRiskAckForArm = false;
            runGpsDropDetected = false;
            runGpsDropNotified = false;
            resetRunGpsTelemetry();
            runReady = false;
            suppressAutoArm = true;
            runDisplayStartPerf = 0;
            elSpeed.textContent = '0';
            elRpm.textContent = '0';
            elHp.textContent = '0';
            elTorque.textContent = '0';
            elHpWheel.textContent = '0';
            elHpCrank.textContent = '0';
            elHpCorr.textContent = '0';
            elThrottle.textContent = '0';
            elFuelRate.textContent = '0.0';
            drawLiveGauges(0, 0, 0, 0);
            msPowerWheel.textContent = '0';
            msG.textContent = '0.00';
            saveCurrentRun(mode, { quietVoice: true });
            releaseMeasurementModeForNewPick();
            vibrate([160, 80, 160]);
            flash('rgba(0,255,140,0.55)', 260);
          }
          if (runArmed) {
            if (runActive && mode === '__track_nav__' && trackStartRunLapArm) {
              const te = (sample.t_ms - trackLapStartTms) / 1000.0;
              elAutoRunInfo.textContent = 'Track lap: RUNNING (' + te.toFixed(1) + ' s)';
              elAutoRunReasonInfo.textContent = 'Return to saved line & stop (≤' + TRACK_STANDSTILL_KMH + ' km/h). Pass without stopping = no finish; stop on line to close lap.';
            } else if (!runActive && mode === '__track_nav__' && trackStartRunLapArm && trackAwaitingSpeedForLap) {
              elAutoRunInfo.textContent = 'Track: ARMED — waiting speed ≥ ' + cachedAutoArmKmh.toFixed(0) + ' km/h';
              elAutoRunReasonInfo.textContent = 'Threshold from Settings (Auto-arm GPS). Lap time starts when you exceed it; next lap same after you stop on the line.';
            } else if (runActive) {
              const modeLabel = modeDisplayLabel(mode);
              elAutoRunInfo.textContent = 'AutoRun: running ' + modeLabel + ' | d=' + runDistanceM.toFixed(1) + 'm';
              elAutoRunReasonInfo.textContent = 'Reason: run in progress';
            } else {
              elAutoRunInfo.textContent = 'AutoRun: armed (' + modeDisplayLabel(mode) + ')';
              if (mode !== 'dyno_pull' && !gpsOkForAccelModes) {
                elAutoRunReasonInfo.textContent = 'Reason: GNSS weak and GPS-heavy speed fusion — improve sky view, or wait for OBD-weighted speed.';
              } else if (!cr.valid && (mode === 'drag_custom' || mode === 'mid_custom')) {
                elAutoRunReasonInfo.textContent = 'Reason: Set From & To km/h (From must be less than To)';
              } else if (mode === 'drag_custom_dist' && !cd.valid) {
                elAutoRunReasonInfo.textContent = 'Reason: Enter distance in metres (5-5000), or pick 1/8 / 1/4 / 1/2 mile above';
              } else if ((mode === 'drag_0_100' || mode === 'drag_0_200') && !runReady) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting standstill (<3 km/h) before start';
              } else if (mode === 'drag_custom' && cr.lo < 15 && !runReady) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting standstill (<3 km/h) before start';
              } else if (mode === 'drag_0_100' && sample.speed_kmh < 5) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting start threshold (>=5 km/h)';
              } else if (mode === 'drag_0_200' && sample.speed_kmh < 5) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting start threshold (>=5 km/h)';
              } else if (mode === 'drag_custom' && cr.lo < 15 && sample.speed_kmh < 5) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting start threshold (>=5 km/h)';
              } else if (mode === 'mid_60_100' && sample.speed_kmh < 60) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting speed >= 60 km/h';
              } else if (mode === 'mid_80_120' && sample.speed_kmh < 80) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting speed >= 80 km/h';
              } else if (mode === 'mid_custom' && cr.valid && sample.speed_kmh < cr.lo) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting speed >= ' + cr.lo + ' km/h';
              } else if (mode === 'drag_custom' && cr.lo >= 15 && sample.speed_kmh < cr.lo) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting speed >= ' + cr.lo + ' km/h';
              } else if (mode === 'mid_100_200' && sample.speed_kmh < 100) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting speed >= 100 km/h';
              } else if (mode === 'braking_100_0' && sample.speed_kmh < 100) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting speed >= 100 km/h before braking';
              } else if (mode === 'braking_custom' && parseBrakingRange().valid && sample.speed_kmh < parseBrakingRange().from) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting speed >= ' + parseBrakingRange().from + ' km/h before braking';
              } else if (mode === 'dyno_pull' && sample.rpm < 1800) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting Dyno mode start RPM';
              } else if (isRollingThrottleMode(mode) && sample.throttle_pct <= 25) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting throttle > 25%';
              } else if ((mode === 'drag_201m' || mode === 'drag_402m' || mode === 'drag_804m' || mode === 'drag_custom_dist') && sample.speed_kmh < 5) {
                elAutoRunReasonInfo.textContent = 'Reason: Waiting start threshold (>=5 km/h)';
              } else {
                elAutoRunReasonInfo.textContent = 'Reason: Ready to start';
              }
            }
            prevRunSample = sample;
          } else {
            elAutoRunInfo.textContent = 'AutoRun: idle';
            elAutoRunReasonInfo.textContent = measurementAutoArmPref
              ? 'Reason: Select a mode below to arm (dashboard is always live)'
              : 'Reason: Select a mode, then press START RUN to arm (dashboard is always live)';
          }

          const nowPaint = performance.now();
          if (nowPaint - lastUiPaintMs >= 45) {
            lastUiPaintMs = nowPaint;
            paintHomeCardsFromMsg(msg);
            if (!isSwitchingToHome) {
              paintMeasurementGauges(msg);
            }
            updateBatteryWidget(msg);
          }
        };
      }

      btnAbort.addEventListener('click', () => {
        if (btnAbort.disabled) return;
        abortRun('Manual abort');
      });
      if (btnBackHome) btnBackHome.addEventListener('click', () => setScreen('home'));
      function goToResultsScreen() {
        setScreen('results');
      }
      if (btnResultsScreen) btnResultsScreen.addEventListener('click', goToResultsScreen);
      if (btnStartRun) btnStartRun.addEventListener('click', () => {
        if (runActive) return;
        if (lastLiveMsg && !lastLiveMsg.coast_cal_valid && !lastLiveMsg.coast_bypass) {
          showDtModal('Coast-down calibration is required for accurate power measurements. Complete calibration first or go to Settings to bypass it.');
          btnStartRun.classList.add('btnStartRun--invalid');
          setTimeout(() => { btnStartRun.classList.remove('btnStartRun--invalid'); }, 560);
          return;
        }
        const mode = getMeasurementMode();
        if (mode === '__track_nav__') {
          if (runArmed) return;
          if (!trackSessionActive) {
            showDtModal('Start a track session first — choose TRACK (laps) from the menu (session starts automatically).');
            btnStartRun.classList.add('btnStartRun--invalid');
            setTimeout(() => { btnStartRun.classList.remove('btnStartRun--invalid'); }, 560);
            return;
          }
          const lm = lastLiveMsg;
          if (!lm || !lm.gps_lock) {
            showDtModal('GPS lock required for track laps. Wait for a fix on the start/finish line.');
            btnStartRun.classList.add('btnStartRun--invalid');
            setTimeout(() => { btnStartRun.classList.remove('btnStartRun--invalid'); }, 560);
            return;
          }
          const la = Number(lm.gps_lat || 0);
          const lo = Number(lm.gps_lon || 0);
          if (Math.abs(la) < 1e-5 && Math.abs(lo) < 1e-5) {
            showDtModal('GPS position not valid yet — wait for lock on the line.');
            btnStartRun.classList.add('btnStartRun--invalid');
            setTimeout(() => { btnStartRun.classList.remove('btnStartRun--invalid'); }, 560);
            return;
          }
          if (lastMissingFields.length) {
            showDtModal('Complete required setup fields first (weight, tire, etc).');
            btnStartRun.classList.add('btnStartRun--invalid');
            setTimeout(() => { btnStartRun.classList.remove('btnStartRun--invalid'); }, 560);
            return;
          }
          trackGateLat = la;
          trackGateLon = lo;
          trackStartRunLapArm = true;
          trackAwaitingSpeedForLap = true;
          trackRunHasLeftGate = false;
          runArmed = true;
          runActive = false;
          suppressAutoArm = true;
          allowManualStartRun = false;
          elAutoRunInfo.textContent = 'Track: ARMED at gate';
          elAutoRunReasonInfo.textContent = 'Exceed ' + cachedAutoArmKmh.toFixed(0) + ' km/h to start lap (Settings → Auto-arm GPS).';
          refreshInteractionLock();
          refreshStartRunButton();
          updateTrackSmartHint();
          return;
        }
        if (runArmed) return;
        if (!allowManualStartRun) return;
        if (!mode) {
          showDtModal('Please select a measurement mode before starting.');
          btnStartRun.classList.add('btnStartRun--invalid');
          setTimeout(() => { btnStartRun.classList.remove('btnStartRun--invalid'); }, 560);
          return;
        }
        if (lastMissingFields.length) {
          showDtModal('Complete required setup fields first (weight, tire, etc).');
          btnStartRun.classList.add('btnStartRun--invalid');
          setTimeout(() => { btnStartRun.classList.remove('btnStartRun--invalid'); }, 560);
          return;
        }
        const armManualRun = () => {
          allowManualStartRun = false;
          runArmed = true;
          runReady = true;
          suppressAutoArm = false;
          sessionPeakPower = 0;
          sessionPeakTorque = 0;
          elAutoRunInfo.textContent = 'AutoRun: armed (manual start request)';
          elAutoRunReasonInfo.textContent = 'Reason: Manual start armed — begin driving.';
          refreshInteractionLock();
        };
        if (isGpsRiskForCurrentMode(mode) && !gpsRiskAckForArm) {
          showGpsRiskAcknowledgeModal(() => {
            gpsRiskAckForArm = true;
            beep(740, 1300);
            armManualRun();
          });
          return;
        }
        armManualRun();
      });
      if (btnStartRun) {
        const pressOn = () => { btnStartRun.classList.add('btnStartRun--pressed'); };
        const pressOff = () => { btnStartRun.classList.remove('btnStartRun--pressed'); };
        btnStartRun.addEventListener('pointerdown', pressOn);
        btnStartRun.addEventListener('pointerup', pressOff);
        btnStartRun.addEventListener('pointerleave', pressOff);
        btnStartRun.addEventListener('pointercancel', pressOff);
      }
      if (btnTrackStart) btnTrackStart.addEventListener('click', () => {
        if (getMeasurementMode() === '__track_nav__' && (runArmed || runActive)) {
          abortRun('Track session reset');
        }
        clearTrackStartRunArmState();
        trackSessionActive = true;
        trackBestLapS = 0;
        trackLaps = [];
        resetTrackLineAnchor();
        renderTrackLaps();
        updateTrackHeader();
        if (getMeasurementMode() === '__track_nav__') syncTrackSessionToMeasurementSummary();
      });
      if (btnTrackStop) btnTrackStop.addEventListener('click', () => {
        trackSessionActive = false;
        updateTrackHeader();
        if (getMeasurementMode() === '__track_nav__') syncTrackSessionToMeasurementSummary();
      });
      if (btnTrackExportCsv) btnTrackExportCsv.addEventListener('click', () => {
        const rows = ['lap_number,time_s,top_speed_kmh,avg_speed_kmh,best_lap'];
        trackLaps.forEach(l => rows.push([l.lap_number, l.time.toFixed(3), l.top_speed.toFixed(2), l.avg_speed.toFixed(2), l.best_lap ? '1' : '0'].join(',')));
        const blob = new Blob([rows.join('\n')], { type: 'text/csv' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'dynotrack_track_laps.csv';
        a.click();
      });
      if (btnTrackPanelClose) btnTrackPanelClose.addEventListener('click', () => {
        if (getMeasurementMode() === '__track_nav__' && (runArmed || runActive)) {
          abortRun('Track panel closed');
        }
        clearTrackStartRunArmState();
        trackPanelVisible = false;
        trackSessionActive = false;
        if (trackModePanel) trackModePanel.classList.remove('visible');
        updateTrackHeader();
        const backMode = measurementModeBeforeTrack || '';
        measurementModeBeforeTrack = '';
        committedMeasurementMode = backMode;
        if (measurementModeEl) {
          if (backMode) measurementModeEl.setAttribute('data-last', backMode);
          else measurementModeEl.removeAttribute('data-last');
        }
        restoreSummarySnapshotIfAny();
        renderMeasurementResult();
        applyCustomModeArmPolicy(getMeasurementMode());
        refreshInteractionLock();
      });
      if (measurementModeEl) measurementModeEl.addEventListener('change', () => {
        const picked = measurementModeEl.value;
        gpsRiskAckForArm = false;
        if (picked === '__track_nav__') {
          trackPanelVisible = true;
          if (trackModePanel) trackModePanel.classList.add('visible');
          enterCommittedTrackMode();
          resetMeasurementModeSelectDisplay();
          refreshModeRequiredStyle();
          updateCustomRangeVisibility();
          updateResultsLayout();
          refreshInteractionLock();
          try { measurementModeEl.blur(); } catch (e) {}
          if (activeScreen !== 'home') setScreen('home');
          beginTrackPanelSessionAndScroll();
          return;
        }
        if (picked === '') {
          if (runActive || runArmed) {
            resetMeasurementModeSelectDisplay();
            try { measurementModeEl.blur(); } catch (e) {}
            return;
          }
          leaveCommittedTrackModeToMenuChoice('');
          committedMeasurementMode = '';
          customApplySealActive = false;
          measurementModeEl.removeAttribute('data-last');
          trackPanelVisible = false;
          trackSessionActive = false;
          if (trackModePanel) trackModePanel.classList.remove('visible');
          refreshModeRequiredStyle();
          updateCustomRangeVisibility();
          updateResultsLayout();
          applyCustomModeArmPolicy('');
          refreshInteractionLock();
          updateTrackHeader();
          renderMeasurementResult();
          try { measurementModeEl.blur(); } catch (e) {}
          return;
        }
        if (runActive || runArmed) {
          resetMeasurementModeSelectDisplay();
          try { measurementModeEl.blur(); } catch (e) {}
          updateCustomRangeVisibility();
          return;
        }
        leaveCommittedTrackModeToMenuChoice(picked);
        trackPanelVisible = false;
        trackSessionActive = false;
        if (trackModePanel) trackModePanel.classList.remove('visible');
        updateTrackHeader();
        allowManualStartRun = false;
        customApplySealActive = false;
        committedMeasurementMode = picked;
        measurementModeEl.setAttribute('data-last', picked);
        refreshModeRequiredStyle();
        const m = picked;
        if (m === 'mid_custom') {
          customStartEl.value = '80';
          customEndEl.value = '120';
        } else if (m === 'drag_custom') {
          customStartEl.value = '0';
          customEndEl.value = '100';
        } else if (m === 'braking_custom') {
          const fromN = parseInt(customStartEl.value, 10);
          const toN = parseInt(customEndEl.value, 10);
          if (!isFinite(fromN) || !isFinite(toN) || fromN <= toN) {
            customStartEl.value = '80';
            customEndEl.value = '20';
          }
        }
        updateCustomRangeVisibility();
        updateResultsLayout();
        normalizeCustomInputs();
        applyCustomModeArmPolicy(m);
        refreshInteractionLock();
        renderMeasurementResult();
        resetMeasurementModeSelectDisplay();
        try { measurementModeEl.blur(); } catch (e) {}
      });
      if (customStartEl) customStartEl.addEventListener('keydown', onCustomFieldEnter);
      if (customEndEl) customEndEl.addEventListener('keydown', onCustomFieldEnter);
      if (customDragDistMEl) customDragDistMEl.addEventListener('keydown', onCustomFieldEnter);
      const btnCustomSpeedApply = document.getElementById('btnCustomSpeedApply');
      const btnCustomDistApply = document.getElementById('btnCustomDistApply');
      if (btnCustomSpeedApply) btnCustomSpeedApply.addEventListener('click', () => applyCustomFieldsNow(btnCustomSpeedApply));
      if (btnCustomDistApply) btnCustomDistApply.addEventListener('click', () => applyCustomFieldsNow(btnCustomDistApply));
      const btnMeasurementClose = document.getElementById('btnMeasurementClose');
      if (btnMeasurementClose) btnMeasurementClose.addEventListener('click', () => releaseMeasurementModeForNewPick());
      commitCustomAccelMidFromInputs();
      commitCustomDistFromInputs();
      btnExportCsv.addEventListener('click', exportRunsCsv);

      btnPrintReport.addEventListener('click', async function () {
        const s = lastMeasurementSummary;
        const u = powerUnitLabel();
        const hasRun = s.modeKey && s.modeKey !== '-' && s.status && s.status !== 'idle';
        if (!hasRun) {
          showDtModal('No measurement result yet. Open Track for a live session summary, or finish a drag / rolling / braking / dyno run first.');
          return;
        }

        if (isMobileTouchNarrowUi()) {
          showDtModal(
            'Print Report is meant for a laptop or desktop browser. On phones, the system print dialog often drops charts, misaligns tables, or saves a blank page — that is a limitation of mobile browsers, not your run data.\n\n'
            + 'Easy options:\n'
            + '• Open this same dashboard on a PC or Mac (join the DynoTrack‑X Wi‑Fi, same as your phone), then tap Print Report there.\n'
            + '• Or use Export Data / Export Track CSV to save files you can open, archive, or print from a computer.\n\n'
            + 'Your results on the Results screen are complete; printing them cleanly just works better on a larger screen.',
            null
          );
          return;
        }

        if (s.status === 'aborted') {
          const dt = new Date().toLocaleString();
          const modeTxt = escapeHtml(s.mode || '-');
          const printLogoSrc = (typeof location !== 'undefined' && location.origin)
            ? (location.origin + '/logo.png')
            : '/logo.png';
          let logoInlineSrc = printLogoSrc;
          try {
            const r = await fetch(printLogoSrc, { cache: 'force-cache', credentials: 'same-origin' });
            if (r.ok) {
              const blob = await r.blob();
              logoInlineSrc = await new Promise(function (resolve, reject) {
                const fr = new FileReader();
                fr.onload = function () { resolve(fr.result); };
                fr.onerror = reject;
                fr.readAsDataURL(blob);
              });
            }
          } catch (e) { /* keep URL */ }
          const logoSrcAttr = String(logoInlineSrc).replace(/&/g, '&amp;').replace(/"/g, '&quot;');
          const printHeadBlock = '<div class="pr-head" style="display:table;width:100%;table-layout:fixed;margin:0 0 18px 0;">'
            + '<div style="display:table-row;">'
            + '<div style="display:table-cell;vertical-align:top;text-align:left;padding:0 14px 0 0;width:62%;">'
            + '<div style="font-size:21px;font-weight:800;line-height:1.25;margin:0 0 10px 0;">DynoTrack X — Report</div>'
            + '<div style="font-size:12px;color:#333;">Generated: ' + escapeHtml(dt) + '</div>'
            + '</div>'
            + '<div style="display:table-cell;vertical-align:top;text-align:right;width:38%;overflow:visible;">'
            + '<img class="pr-logo" src="' + logoSrcAttr + '" alt="DynoTrack X" style="display:inline-block;max-height:90px;height:auto;width:auto;max-width:260px;object-fit:contain;vertical-align:top;"/>'
            + '</div>'
            + '</div>'
            + '</div>';
          const printBody = printHeadBlock
            + '<h3 style="font-size:15px;">Results</h3>'
            + '<p style="font-size:13px;line-height:1.45;color:#333;margin:0 0 14px 0;"><strong>Status: Aborted</strong></p>'
            + '<p style="font-size:12px;color:#555;margin:0 0 16px 0;">The run was stopped before completion. No power, speed, distance, or timing figures are shown — they would not represent a valid measurement.</p>'
            + '<table style="width:100%;border-collapse:collapse;font-size:12px;">'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;width:38%;">Mode</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + modeTxt + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Outcome</td><td style="padding:6px;border-bottom:1px solid #ccc;">Aborted — no valid run data</td></tr>'
            + '</table>';
          const reportHtml = '<!DOCTYPE html><html><head><meta charset="utf-8"/><title>\u200B</title>'
            + '<style>'
            + 'body{margin:0;padding:16px;font-family:Arial,Helvetica,sans-serif;color:#111;-webkit-print-color-adjust:exact;print-color-adjust:exact;}'
            + '@media print{@page{margin:0}body{padding:12mm}.pr-logo{max-height:100px!important;height:auto!important;width:auto!important;max-width:280px!important;}}'
            + '</style></head><body>'
            + printBody
            + '</body></html>';
          const iframe = document.createElement('iframe');
          iframe.setAttribute('title', 'DynoTrack print report');
          iframe.setAttribute('aria-hidden', 'true');
          const printIframeW = Math.max(900, Math.min(1600, (window.innerWidth || 900)));
          iframe.style.cssText = 'position:fixed;left:-4000px;top:0;width:' + printIframeW + 'px;height:1400px;border:0;opacity:0;pointer-events:none;overflow:hidden';
          document.body.appendChild(iframe);
          const idoc = iframe.contentDocument || iframe.contentWindow.document;
          idoc.open();
          idoc.write(reportHtml);
          idoc.close();
          const iw = iframe.contentWindow;
          let printDone = false;
          const finishPrint = () => {
            if (printDone) return;
            printDone = true;
            try { iframe.remove(); } catch (e) {}
            setScreen('home');
          };
          iw.addEventListener('afterprint', finishPrint);
          requestAnimationFrame(() => {
            requestAnimationFrame(() => {
              try {
                iw.focus();
                iw.print();
              } catch (e) {
                finishPrint();
              }
            });
          });
          setTimeout(() => {
            window.addEventListener('focus', function printFocusAbortFallback() {
              window.removeEventListener('focus', printFocusAbortFallback);
              if (!printDone) setTimeout(finishPrint, 400);
            });
          }, 600);
          return;
        }

        const isTrackRun = s.modeKey === '__track_nav__';
        const dt = new Date().toLocaleString();
        const modeTxt = escapeHtml(s.mode || '-');
        const statusTxt = escapeHtml(s.status || '-');
        const timeTxt = isTrackRun
          ? (isFinite(s.timeS) ? (s.timeS.toFixed(2) + ' s (best lap)') : '—')
          : (isFinite(s.timeS) ? (s.timeS.toFixed(2) + ' s') : '-');
        const distTxt = isFinite(s.distanceM) ? (s.distanceM.toFixed(1) + ' m') : '-';
        const speedWindowTxt = isTrackRun
          ? '— (lap session)'
          : ((isFinite(s.startKmh) && isFinite(s.endKmh))
            ? (s.startKmh.toFixed(1) + ' → ' + s.endKmh.toFixed(1) + ' km/h')
            : '-');
        const spdTxt = isTrackRun
          ? ((isFinite(s.avgSpeedKmh) && isFinite(s.maxSpeedKmh))
            ? ('Across laps: ' + s.avgSpeedKmh.toFixed(1) + ' avg / ' + s.maxSpeedKmh.toFixed(1) + ' max / '
              + (isFinite(s.minSpeedKmh) ? s.minSpeedKmh.toFixed(1) : '-') + ' min km/h (per-lap top speeds)')
            : '—')
          : ((isFinite(s.avgSpeedKmh) && isFinite(s.maxSpeedKmh))
            ? (s.avgSpeedKmh.toFixed(1) + ' avg / ' + s.maxSpeedKmh.toFixed(1) + ' max / '
              + (isFinite(s.minSpeedKmh) ? s.minSpeedKmh.toFixed(1) : '-') + ' min km/h')
            : '-');
        let peakPowerTxt = '-';
        if (isFinite(s.peakHp)) {
          peakPowerTxt = isFinite(s.peakHpCorrRpm)
            ? s.peakHp.toFixed(1) + ' ' + u + ' (corr.) @ ' + Math.round(s.peakHpCorrRpm) + ' rpm'
            : s.peakHp.toFixed(1) + ' ' + u + ' (corr.)';
        }
        let peakTorqueTxt = '-';
        if (isFinite(s.peakTorqueNm)) {
          peakTorqueTxt = isFinite(s.peakTorqueRpm)
            ? s.peakTorqueNm.toFixed(1) + ' Nm @ ' + Math.round(s.peakTorqueRpm) + ' rpm'
            : s.peakTorqueNm.toFixed(1) + ' Nm';
        }
        const peakRpmTxt = isFinite(s.peakRpm) ? (String(Math.round(s.peakRpm)) + ' rpm') : '-';
        const rpmWinTxt = (isFinite(s.startRpm) && isFinite(s.endRpm))
          ? (Math.round(s.startRpm) + ' → ' + Math.round(s.endRpm) + ' rpm')
          : '-';
        const maxThrTxt = isFinite(s.maxThrottle) ? (s.maxThrottle.toFixed(0) + ' %') : '-';
        const pf = (v) => (isFinite(v) ? (v.toFixed(1) + ' ' + u) : '-');
        const pwrSplitTxt = (pf(s.peakHpWheel) !== '-' || pf(s.peakHpCrank) !== '-' || pf(s.peakHpIndicated) !== '-')
          ? (pf(s.peakHpWheel) + ' / ' + pf(s.peakHpCrank) + ' / ' + pf(s.peakHpIndicated))
          : '-';
        let lossTxt = '-';
        if (isFinite(s.peakLossTotal)) {
          lossTxt = pf(s.peakLossTotal) + ' total';
          if (isFinite(s.peakLossAero) || isFinite(s.peakLossRoll) || isFinite(s.peakLossSlope)) {
            lossTxt += ' (' + pf(s.peakLossAero) + ' aero, ' + pf(s.peakLossRoll) + ' roll, ' + pf(s.peakLossSlope) + ' slope)';
          }
        }
        const slipTxt = isFinite(s.maxSlipPct) ? (s.maxSlipPct.toFixed(1) + ' %') : '-';
        const fuelTxt = (isFinite(s.avgFuelLph) && isFinite(s.maxFuelLph))
          ? (s.avgFuelLph.toFixed(2) + ' / ' + s.maxFuelLph.toFixed(2) + ' L/h')
          : '-';
        const oilTxt = (isFinite(s.engineOilStartC) && isFinite(s.engineOilEndC))
          ? (s.engineOilStartC.toFixed(1) + ' → ' + s.engineOilEndC.toFixed(1) + ' °C')
          : '-';
        const rhoTxt = isFinite(s.airDensity) ? (s.airDensity.toFixed(4) + ' kg/m³') : '-';
        let corrTxt = '-';
        if (s.corrStd && isFinite(s.corrFactorK)) corrTxt = String(s.corrStd).toUpperCase() + ' | K=' + s.corrFactorK.toFixed(3);
        else if (s.corrStd) corrTxt = String(s.corrStd).toUpperCase();
        else if (isFinite(s.corrFactorK)) corrTxt = 'K=' + s.corrFactorK.toFixed(3);
        const drvP = s.driveType ? String(s.driveType).toUpperCase() : '';
        const gbP = isFinite(s.lossGearboxPct) ? (s.lossGearboxPct.toFixed(1) + '%') : '';
        let vehTxt = '-';
        if (drvP && gbP) vehTxt = drvP + ' | gearbox loss ' + gbP;
        else if (drvP) vehTxt = drvP;
        else if (s.vehicleTxt && s.vehicleTxt !== '-') vehTxt = s.vehicleTxt;
        const skipDistance = s.modeKey === 'dyno_pull' || isTrackRun;
        const isDynoRun = s.modeKey === 'dyno_pull';
        // Use lastMeasurementSummary only (same as main results table). DOM fallbacks differ on mobile layout.
        const dynoPeakP = peakPowerTxt;
        const dynoPeakT = peakTorqueTxt;
        const dynoWhp = pwrSplitTxt;
        const dynoLossLine = lossTxt;
        const dynoGb = isFinite(s.lossGearboxPct) ? (s.lossGearboxPct.toFixed(1) + '%') : '-';
        const dynoCorr = corrTxt;
        const dynoSlipFuel = slipTxt + ' / ' + (fuelTxt === '-' ? '—' : fuelTxt);
        const dynoAmb = s.ambientTxt || '-';
        const dynoRows = isDynoRun
          ? (
            '<h3>Dyno graph summary</h3>'
            + '<table style="width:100%;border-collapse:collapse;font-size:12px;">'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Peak power (corr.) @ RPM</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoPeakP) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Peak torque @ RPM</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoPeakT) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">WHP vs engine @ peak corr.</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoWhp) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Loss @ peak corr.</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoLossLine) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Gearbox loss</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoGb) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Correction</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoCorr) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Max slip / fuel (avg · max)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoSlipFuel) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Ambient (T/P/H)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(dynoAmb) + '</td></tr>'
            + '</table>'
            + '<p style="font-size:11px;color:#444;margin-top:8px;">Crank power is estimated from drivetrain and road-load losses.</p>'
          )
          : '';
        let dynoGraphPrintHtml = '';
        if (isDynoRun) {
          let ptsP = getPointsForExportedRun();
          if (ptsP.length >= 2) {
            const maxP = 520;
            if (ptsP.length > maxP) {
              const step = Math.ceil(ptsP.length / maxP);
              const thin = [];
              for (let i = 0; i < ptsP.length; i += step) thin.push(ptsP[i]);
              if (thin.length < 2) thin.push(ptsP[ptsP.length - 1]);
              ptsP = thin;
            }
            try {
              const pcv = document.createElement('canvas');
              pcv.setAttribute('aria-hidden', 'true');
              pcv.style.cssText = 'position:fixed;left:0;top:0;width:900px;height:280px;opacity:0.02;pointer-events:none;z-index:2147483647;';
              document.body.appendChild(pcv);
              let du = '';
              try {
                drawCurveOn(pcv, ptsP, { w: 900, h: 280, dpr: 2 });
                du = pcv.toDataURL('image/png');
              } finally {
                document.body.removeChild(pcv);
              }
              if (du && du.length > 32 && du.indexOf('data:image/png') === 0) {
                const su = String(du).replace(/&/g, '&amp;').replace(/"/g, '&quot;');
                dynoGraphPrintHtml = '<h3 style="font-size:15px;margin-top:0;">Dyno graph (RPM vs power / torque / loss)</h3>'
                  + '<div style="page-break-inside:avoid;margin:6px 0 16px 0;border:1px solid #333;padding:8px;background:#f6f6f6;">'
                  + '<img src="' + su + '" alt="Dyno curves" style="display:block;width:100%;max-width:920px;height:auto;"/>'
                  + '</div>';
              }
            } catch (e) {}
            if (!dynoGraphPrintHtml) {
              dynoGraphPrintHtml = '<h3 style="font-size:15px;margin-top:0;">Dyno graph (RPM vs power / torque / loss)</h3>'
                + '<p style="font-size:12px;color:#555;margin:8px 0 14px 0;">Graph image could not be captured for print on this device. Summary tables still reflect the last run; use <b>EXPORT DATA</b> for full curves.</p>';
            }
          } else {
            dynoGraphPrintHtml = '<h3 style="font-size:15px;margin-top:0;">Dyno graph (RPM vs power / torque / loss)</h3>'
              + '<p style="font-size:12px;color:#555;margin:8px 0 14px 0;">Graph could not be generated — not enough samples in memory (e.g. buffer cleared). Summary table below still reflects the last run; use <b>EXPORT DATA</b> to keep full curves.</p>';
          }
        }
        const footnote = '<p style="font-size:11px;color:#444;margin-top:10px;">Power/torque figures use the correction selected in settings. Road modes use GPS/OBD fusion as shown in the live app.</p>';
        let liveSnapBlock = '';
        if (lastLiveMsg && (lastLiveMsg.t_ms !== undefined || lastLiveMsg.speed_kmh !== undefined)) {
          const lm = lastLiveMsg;
          liveSnapBlock = '<h3 style="font-size:15px;margin-top:18px;">Last live sample (reference)</h3>'
            + '<table style="width:100%;border-collapse:collapse;font-size:12px;">'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;width:38%;">Speed (fused)</td><td style="padding:6px;border-bottom:1px solid #ccc;">'
            + escapeHtml(Number(lm.speed_kmh || 0).toFixed(1)) + ' km/h</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Speed OBD / GPS</td><td style="padding:6px;border-bottom:1px solid #ccc;">'
            + escapeHtml(Number(lm.speed_obd_kmh || 0).toFixed(1)) + ' / ' + escapeHtml(Number(lm.speed_gps_kmh || 0).toFixed(1)) + ' km/h</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">RPM</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(String(Math.round(Number(lm.rpm || 0)))) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Throttle</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(Number(lm.throttle_pct || 0).toFixed(0)) + ' %</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Power corrected / torque</td><td style="padding:6px;border-bottom:1px solid #ccc;">'
            + escapeHtml(powerConvert(Number(lm.hp_corrected || 0)).toFixed(1)) + ' ' + u + ' / ' + escapeHtml(Number(lm.torque_nm || 0).toFixed(0)) + ' Nm</td></tr>'
            + '</table>';
        }
        const printLogoSrc = (typeof location !== 'undefined' && location.origin)
          ? (location.origin + '/logo.png')
          : '/logo.png';
        let logoInlineSrc = printLogoSrc;
        try {
          const r = await fetch(printLogoSrc, { cache: 'force-cache', credentials: 'same-origin' });
          if (r.ok) {
            const blob = await r.blob();
            logoInlineSrc = await new Promise(function (resolve, reject) {
              const fr = new FileReader();
              fr.onload = function () { resolve(fr.result); };
              fr.onerror = reject;
              fr.readAsDataURL(blob);
            });
          }
        } catch (e) { /* keep URL */ }
        const logoSrcAttr = String(logoInlineSrc).replace(/&/g, '&amp;').replace(/"/g, '&quot;');
        let printMainTable;
        if (isTrackRun) {
          const nL = trackLaps.length;
          let sumT = 0;
          trackLaps.forEach(l => { sumT += l.time; });
          const avgLStr = nL > 0 ? ((sumT / nL).toFixed(2) + ' s') : '—';
          const lastL = nL ? trackLaps[nL - 1] : null;
          const lastStr = lastL ? (lastL.time.toFixed(2) + ' s') : '—';
          const bestStr = trackBestLapS > 0 ? (trackBestLapS.toFixed(2) + ' s') : '—';
          printMainTable = '<table style="width:100%;border-collapse:collapse;font-size:12px;">'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;width:38%;">Mode</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + modeTxt + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Status</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + statusTxt + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Laps recorded</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(String(nL)) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Best lap</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(bestStr) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Average lap</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(avgLStr) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Last lap</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(lastStr) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Duration (best lap)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(timeTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Speed stats (laps)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(spdTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Ambient (IAT / P / RH)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(s.ambientTxt || '-') + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">GPS</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(s.gpsTxt || '-') + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Drive / gearbox loss</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(vehTxt) + '</td></tr>'
            + '</table>'
            + '<p style="font-size:11px;color:#444;margin-top:8px;">Per-lap table: use <b>EXPORT TRACK CSV</b> in the track panel on the dashboard.</p>';
        } else {
          printMainTable = '<table style="width:100%;border-collapse:collapse;font-size:12px;">'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;width:38%;">Mode</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + modeTxt + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Status</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + statusTxt + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Duration</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(timeTxt) + '</td></tr>'
            + (skipDistance ? '' : '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Distance</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(distTxt) + '</td></tr>')
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Speed window (start → end)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(speedWindowTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Avg / max / min speed</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(spdTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Peak power (corr.) @ RPM</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(peakPowerTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Peak torque @ RPM</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(peakTorqueTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Max RPM (during run)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(peakRpmTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">RPM (start → end)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(rpmWinTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Max throttle</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(maxThrTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Power @ peak corr. (WHP / crank / indicated)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(pwrSplitTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Road-load losses @ peak corr.</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(lossTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Max tire slip</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(slipTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Fuel rate (avg / max)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(fuelTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Engine oil (start → end)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(oilTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Air density</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(rhoTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Correction (std / K)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(corrTxt) + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Ambient (IAT / P / RH)</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(s.ambientTxt || '-') + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">GPS</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(s.gpsTxt || '-') + '</td></tr>'
            + '<tr><td style="padding:6px;border-bottom:1px solid #ccc;">Drive / gearbox loss</td><td style="padding:6px;border-bottom:1px solid #ccc;">' + escapeHtml(vehTxt) + '</td></tr>'
            + '</table>';
        }
        const printHeadBlock = '<!-- If the browser adds a duplicate title line in the header, turn off Headers and footers in the print dialog. -->'
          + '<div class="pr-head" style="display:table;width:100%;table-layout:fixed;margin:0 0 18px 0;">'
          + '<div style="display:table-row;">'
          + '<div style="display:table-cell;vertical-align:top;text-align:left;padding:0 14px 0 0;width:62%;">'
          + '<div style="font-size:21px;font-weight:800;line-height:1.25;margin:0 0 10px 0;">DynoTrack X — Report</div>'
          + '<div style="font-size:12px;color:#333;">Generated: ' + escapeHtml(dt) + '</div>'
          + '</div>'
          + '<div style="display:table-cell;vertical-align:top;text-align:right;width:38%;overflow:visible;">'
          + '<img class="pr-logo" src="' + logoSrcAttr + '" alt="DynoTrack X" style="display:inline-block;max-height:90px;height:auto;width:auto;max-width:260px;object-fit:contain;vertical-align:top;"/>'
          + '</div>'
          + '</div>'
          + '</div>';
        const printBodyDynoPaged = printHeadBlock
          + '<h3 style="font-size:15px;">Results</h3>'
          + printMainTable
          + liveSnapBlock
          + footnote
          + '<div class="pr-dyno-page2" style="page-break-before:always;break-before:page;padding-top:3em;">'
          + dynoGraphPrintHtml
          + dynoRows
          + '</div>';
        const printBodyDefault = printHeadBlock
          + '<h3 style="font-size:15px;">Results</h3>'
          + printMainTable
          + dynoGraphPrintHtml
          + dynoRows
          + liveSnapBlock
          + footnote;
        const reportHtml = '<!DOCTYPE html><html><head><meta charset="utf-8"/><title>\u200B</title>'
          + '<style>'
          + 'body{margin:0;padding:16px;font-family:Arial,Helvetica,sans-serif;color:#111;-webkit-print-color-adjust:exact;print-color-adjust:exact;}'
          + '.pr-dyno-page2{page-break-before:always;break-before:page;padding-top:3em;}'
          + '@media print{@page{margin:0}body{padding:12mm}.pr-logo{max-height:100px!important;height:auto!important;width:auto!important;max-width:280px!important;}.pr-dyno-page2{padding-top:14mm!important;}}'
          + '</style></head><body>'
          + (isDynoRun ? printBodyDynoPaged : printBodyDefault)
          + '</body></html>';
        const iframe = document.createElement('iframe');
        iframe.setAttribute('title', 'DynoTrack print report');
        iframe.setAttribute('aria-hidden', 'true');
        const printIframeW = Math.max(900, Math.min(1600, (window.innerWidth || 900)));
        iframe.style.cssText = 'position:fixed;left:-4000px;top:0;width:' + printIframeW + 'px;height:1400px;border:0;opacity:0;pointer-events:none;overflow:hidden';
        document.body.appendChild(iframe);
        const idoc = iframe.contentDocument || iframe.contentWindow.document;
        idoc.open();
        idoc.write(reportHtml);
        idoc.close();
        const iw = iframe.contentWindow;
        let printDone = false;
        const finishPrint = () => {
          if (printDone) return;
          printDone = true;
          try { iframe.remove(); } catch (e) {}
          setScreen('home');
        };
        iw.addEventListener('afterprint', finishPrint);
        requestAnimationFrame(() => {
          requestAnimationFrame(() => {
            try {
              iw.focus();
              iw.print();
            } catch (e) {
              finishPrint();
            }
          });
        });
        // Backup when afterprint is missing (some browsers): focus returns after the print dialog closes.
        setTimeout(() => {
          window.addEventListener('focus', function printFocusFallback() {
            window.removeEventListener('focus', printFocusFallback);
            if (!printDone) setTimeout(finishPrint, 400);
          });
        }, 600);
      });
      if (btnTogglePowerUnit) btnTogglePowerUnit.addEventListener('click', () => {
        if (runActive || runArmed) return;
        powerUnit = (powerUnit === 'hp') ? 'kw' : 'hp';
        btnTogglePowerUnit.textContent = 'Power Unit: ' + (powerUnit === 'hp' ? 'HP' : 'kW');
        refreshPowerUnitLabels();
        if (lastLiveMsg) {
          elHp.textContent = powerConvert(Number(lastLiveMsg.hp)).toFixed(0);
          elHpWheel.textContent = powerConvert(Number(lastLiveMsg.hp_wheel)).toFixed(0);
          elHpCrank.textContent = powerConvert(Number(lastLiveMsg.hp_crank)).toFixed(0);
          elHpCorr.textContent = powerConvert(Number(lastLiveMsg.hp_corrected)).toFixed(0);
          elLossInfo.textContent = 'Loss total: ' + powerConvert(Number(lastLiveMsg.hp_loss_total)).toFixed(1)
            + ' ' + powerUnitLabel();
        }
        if (lastMeasurementSummary.modeKey === 'dyno_pull') {
          const pts = getPointsForExportedRun();
          if (pts.length >= 2) {
            drawCurve(pts);
            updateReportFromPoints(pts, lastLiveMsg || {});
          }
        }
      });

      refreshPowerUnitLabels();
      if (measurementModeEl) {
        const sel = measurementModeEl.value;
        if (sel && sel !== '__track_nav__') {
          committedMeasurementMode = sel;
          measurementModeEl.setAttribute('data-last', sel);
        }
        resetMeasurementModeSelectDisplay();
      }
      refreshModeRequiredStyle();
      updateCustomRangeVisibility();
      updateResultsLayout();
      renderMeasurementResult();
      drawLiveGauges(0, 0, 0, 0);
      refreshInteractionLock();
      setScreen('home');
      updateTrackHeader();
      renderTrackLaps();
      updateRotateHint();
      function redrawGaugesLayout() {
        if (gaugesLayoutRaf != null) return;
        gaugesLayoutRaf = requestAnimationFrame(() => {
          gaugesLayoutRaf = null;
          updateRotateHint();
          if (lastLiveMsg) {
            paintHomeCardsFromMsg(lastLiveMsg);
            paintMeasurementGauges(lastLiveMsg);
          } else drawLiveGauges(0, 0, 0, 0);
          if (activeScreen === 'results') {
            updateResultsLayout();
            refreshDynoResultsCanvas();
          }
        });
      }
      window.addEventListener('resize', redrawGaugesLayout);
      window.addEventListener('orientationchange', redrawGaugesLayout);
      loadRuns();
      setInterval(() => {
        if (elLastLiveInfo && lastWsMsgAt > 0) {
          const age = Date.now() - lastWsMsgAt;
          elLastLiveInfo.textContent = 'Last live update: ' + (age / 1000).toFixed(1) + 's ago';
        }
      }, 1500);
      document.addEventListener('visibilitychange', () => {
        if (document.hidden) return;
        try {
          if (ws && ws.readyState === WebSocket.OPEN) return;
          reconnectDelayMs = 200;
          if (reconnectTimer) {
            clearTimeout(reconnectTimer);
            reconnectTimer = null;
          }
          connectWs();
        } catch (e) {}
      });
      window.addEventListener('online', () => {
        try {
          reconnectDelayMs = 200;
          if (reconnectTimer) {
            clearTimeout(reconnectTimer);
            reconnectTimer = null;
          }
          connectWs();
        } catch (e) {}
      });
      if (btnCalHelp) btnCalHelp.addEventListener('click', () => {
        showDtModal('Coast-down calibration is required for accurate power measurements. It measures aerodynamic drag and rolling resistance by coasting down from speed.\n\nSteps:\n1. Find a safe, straight road with no traffic.\n2. Accelerate to 100 km/h.\n3. Shift to neutral or declutch.\n4. Coast down without touching pedals until speed drops to 80 km/h.\n5. The app will automatically detect and save the calibration.\n\nIf you don\'t want to do calibration, go to Settings and set "Use coast calibration" to OFF. This bypasses the warnings but may reduce accuracy.', null);
      });
      if (btnCalibrate) btnCalibrate.addEventListener('click', () => {
        if (coastCalInProgress) return;
        coastCalInProgress = true;
        calMsg.textContent = 'Calibration in progress. Accelerate to 100 km/h then coast.';
        showDtModal('Calibration started. Accelerate to 100 km/h, then coast down to 80 km/h without touching pedals. The app will automatically detect and complete the calibration.', null);
      });
      (function initPhoneHeaderObdBadgeLayout() {
        function sync() {
          try {
            var narrow = typeof window.matchMedia === 'function' && window.matchMedia('(max-width: 1024px)').matches;
            var touch = (typeof navigator !== 'undefined' && navigator.maxTouchPoints > 0) || ('ontouchstart' in window);
            document.documentElement.classList.toggle('dt-phone-header-badge', narrow && touch);
          } catch (e) {}
        }
        sync();
        window.addEventListener('resize', sync);
        window.addEventListener('orientationchange', sync);
      })();
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
      *, *::before, *::after { box-sizing: border-box; }
      body {
        margin: 0;
        font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
        background: #000000;
        color: #ffffff;
        overflow-x: hidden;
      }
      .wrap { padding: 18px; max-width: 520px; margin: 0 auto; width: 100%; }
      .brandLogoSettings {
        display: block;
        max-width: 300px;
        width: 100%;
        height: auto;
        margin: 0 auto 14px;
      }
      h1 { font-size: 18px; margin: 0 0 10px; }
      .card {
        border: 1px solid rgba(255,255,255,0.22);
        border-radius: 14px;
        padding: 14px;
        background: rgba(255,255,255,0.06);
        min-width: 0;
        max-width: 100%;
      }
      .row { margin: 12px 0; }
      label { display:block; font-size: 12px; opacity: 1; margin-bottom: 6px; color: #f2f6ff; }
      input, select {
        width: 100%;
        max-width: 100%;
        min-width: 0;
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
      .focusField {
        outline: 3px solid #ffd400;
        outline-offset: 2px;
        border-radius: 8px;
        transition: outline-color 250ms ease;
      }
    </style>
  </head>
  <body>
    <div class="wrap">
      <img src="/logo.png" class="brandLogoSettings" width="300" height="106" alt="DYNOTRACK X"/>
      <h1>Setup</h1>
      <div class="card">
        <div class="row">
          <label for="ambientTempNoteC">Ambient temperature °C (information only)</label>
          <input id="ambientTempNoteC" type="number" step="0.1" placeholder="e.g. 24 — not used in calculations"/>
          <div class="msg">Optional note for your records. Does not affect power, air density, or correction.</div>
        </div>

        <div class="row">
          <label for="weightKg">Vehicle weight (kg) *</label>
          <input id="weightKg" type="number" min="1" step="1" placeholder="e.g. 1450"/>
        </div>

        <div class="row">
          <label for="tireSize">Tire size (e.g. 205/55R16) *</label>
          <input id="tireSize" type="text" placeholder="205/55R16"/>
        </div>

        <div class="row">
          <label for="humidityPct">Humidity %</label>
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
          <div class="msg">Baseline: FWD 15%, RWD 16.5%, AWD 22%. Changing <b>Drive type</b> fills the recommended loss here — adjust if needed, then use <b>Save</b> below.</div>
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
          <label for="wheelRadiusCm">Wheel radius (cm)</label>
          <input id="wheelRadiusCm" type="number" step="0.1" placeholder="e.g. 31.5"/>
          <div class="msg">Distance from wheel center to ground.</div>
        </div>

        <div class="row">
          <label for="corrStandard">Power correction standard</label>
          <select id="corrStandard">
            <option value="din" selected>DIN 70020</option>
            <option value="sae">SAE J1349</option>
          </select>
        </div>
        <div class="row">
          <label for="powerUnit">Power unit (UI)</label>
          <select id="powerUnit">
            <option value="hp" selected>HP</option>
            <option value="kw">kW</option>
          </select>
        </div>
        <div class="row">
          <label for="redlineRpm">Engine redline RPM</label>
          <input id="redlineRpm" type="number" step="50" min="3000" max="11000" placeholder="e.g. 6500"/>
        </div>

        <div class="row">
          <label for="coastBypass">Use coast calibration (PRO)</label>
          <select id="coastBypass">
            <option value="0">OFF (bypass invalid-calibration warnings)</option>
            <option value="1" selected>ON (enforce coast calibration validity)</option>
          </select>
        </div>

        <div class="row">
          <label for="measurementAutoArm">Measurement arm</label>
          <select id="measurementAutoArm">
            <option value="1" selected>Auto — arm when you choose a mode (then same start rules)</option>
            <option value="0">Manual — arm only when you press START RUN (same start rules)</option>
          </select>
        </div>

        <div class="row">
          <label for="autoArmKmh">Auto-arm GPS (km/h)</label>
          <input id="autoArmKmh" type="number" min="0" max="200" step="1" value="15" inputmode="numeric" title="Dashboard: auto-arm when speed reaches this after ABORT"/>
          <div class="msg">When <b>Measurement arm</b> is Auto: also re-arms after ABORT when speed reaches this. When Manual, use START RUN instead.</div>
        </div>

        <button id="btnSave">Save</button>
        <div class="msg" id="status">Loading saved settings...</div>
        <div class="msg"><a href="/">Back to Home</a></div>
      </div>
    </div>

    <script>
      function recommendedLossForDriveType(driveType) {
        if (driveType === 'awd') return 22.0;
        if (driveType === 'rwd') return 16.5;
        return 15.0; // fwd default
      }

      async function loadSettings() {
        const res = await fetch('/api/settings');
        const j = await res.json();
        const driveTypeEl = document.getElementById('driveType');
        driveTypeEl.innerHTML = ''
          + '<option value="fwd">FWD</option>'
          + '<option value="rwd">RWD</option>'
          + '<option value="awd">AWD</option>';
        const ambNote = j.ambientTempNoteC;
        document.getElementById('ambientTempNoteC').value = (ambNote != null && isFinite(Number(ambNote))) ? String(ambNote) : '';
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
        document.getElementById('wheelRadiusCm').value = Number(j.wheelRadiusM * 100 || 31.5).toFixed(1);
        document.getElementById('corrStandard').value = (j.corrStandard || 'din');
        document.getElementById('powerUnit').value = (j.powerUnit || 'hp');
        document.getElementById('redlineRpm').value = Number(j.redlineRpm || 6500).toFixed(0);
        document.getElementById('coastBypass').value = j.coastBypass ? '0' : '1';
        document.getElementById('autoArmKmh').value = String(Math.round(Number(j.autoArmKmh != null ? j.autoArmKmh : 15)));
        document.getElementById('measurementAutoArm').value = (j.measurementAutoArm === false || j.measurementAutoArm === 0) ? '0' : '1';
        document.getElementById('status').textContent = j.setup_ok ? 'Setup OK' : 'Setup not complete yet';
        const driveTypeNow = driveTypeEl.value || 'fwd';
        const lossEl = document.getElementById('drivetrainLossPct');
        if (!lossEl.value || Number(lossEl.value) <= 0) {
          lossEl.value = recommendedLossForDriveType(driveTypeNow).toFixed(1);
        }
      }

      async function saveSettings() {
        const weightKg = document.getElementById('weightKg').value;
        const tireSize = document.getElementById('tireSize').value;
        const ambientTempNoteC = document.getElementById('ambientTempNoteC').value;
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
        const wheelRadiusM = document.getElementById('wheelRadiusCm').value / 100;
        const corrStandard = document.getElementById('corrStandard').value;
        const powerUnit = document.getElementById('powerUnit').value;
        const redlineRpm = document.getElementById('redlineRpm').value;
        const coastBypass = document.getElementById('coastBypass').value;
        const autoArmKmh = document.getElementById('autoArmKmh').value;
        const measurementAutoArm = document.getElementById('measurementAutoArm').value;

        const params = new URLSearchParams();
        params.append('weightKg', weightKg);
        params.append('tireSize', tireSize);
        params.append('ambientTempNoteC', ambientTempNoteC);
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
        params.append('powerUnit', powerUnit);
        params.append('redlineRpm', redlineRpm);
        params.append('coastBypass', coastBypass);
        params.append('autoArmKmh', autoArmKmh);
        params.append('measurementAutoArm', measurementAutoArm);

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
      document.getElementById('driveType').addEventListener('change', function() {
        document.getElementById('drivetrainLossPct').value = recommendedLossForDriveType(this.value || 'fwd').toFixed(1);
      });
      loadSettings().catch(() => {
        document.getElementById('status').textContent = 'Could not load settings.';
      });
      const qp = new URLSearchParams(window.location.search);
      const focusId = qp.get('focus');
      if (focusId) {
        setTimeout(() => {
          const el = document.getElementById(focusId);
          if (!el) return;
          try { el.scrollIntoView({ behavior: 'smooth', block: 'center' }); } catch (e) {}
          el.classList.add('focusField');
          el.focus();
          setTimeout(() => el.classList.remove('focusField'), 2600);
        }, 180);
      }
    </script>
  </body>
</html>
)HTML";

// Static buffer: avoid heap String on every WS tick (reduces fragmentation / rare crashes).
static char g_wsLiveJsonBuf[2816];

/** @return JSON length, or 0 on format error */
static int formatLiveJson(uint32_t t_ms) {
  // Deterministic dummy: includes accel plateaus + braking segment so braking modes see realistic speeds.
  float phase = fmodf((float)t_ms, 10000.0f) / 10000.0f;
  float speed_obd_kmh_raw;
  if (phase < 0.12f) {
    speed_obd_kmh_raw = (phase / 0.12f) * 35.0f;
  } else if (phase < 0.28f) {
    float t = (phase - 0.12f) / 0.16f;
    speed_obd_kmh_raw = 35.0f + t * 120.0f;
  } else if (phase < 0.48f) {
    float t = (phase - 0.28f) / 0.20f;
    speed_obd_kmh_raw = 155.0f * (1.0f - t) + 4.0f * t;
  } else {
    float t = (phase - 0.48f) / 0.52f;
    speed_obd_kmh_raw = 4.0f + t * 196.0f;
  }
  if (speed_obd_kmh_raw > 200.0f) speed_obd_kmh_raw = 200.0f;
  if (speed_obd_kmh_raw < 0.0f) speed_obd_kmh_raw = 0.0f;

  float rpm_raw = 800.0f + speed_obd_kmh_raw * 35.0f; // 800..7000 (approx)
  if (rpm_raw > 7000.0f) rpm_raw = 7000.0f;

  float throttleRaw = 8.0f + 78.0f * phase; // 8..86%
  if (throttleRaw > 100.0f) throttleRaw = 100.0f;
  float fuelRateRaw = 1.8f + 0.020f * rpm_raw + 0.08f * throttleRaw; // rough L/h
  int gpsSats = 6 + (int)(8.0f * phase);       // dummy 6..14
  float gpsHdop = 2.8f - 1.6f * phase;         // dummy 2.8..1.2
  if (gpsHdop < 0.8f) gpsHdop = 0.8f;
  bool gpsLock = (gpsSats >= 8 && gpsHdop <= 2.2f);
  const char* gnssMode = gpsLock ? (gpsSats >= 11 ? "RTK_FLOAT" : "GPS_3D") : "NO_LOCK";
  float speed_gps_kmh_raw = speed_obd_kmh_raw * (1.0f + 0.015f * sinf(phase * 6.28318f));
  static float gpsAltM = 245.0f;
  static float gpsAltPrevM = 245.0f;
  gpsAltM = 245.0f + 12.0f * sinf(phase * 6.28318f);

  float speed_obd_kmh = kalmanUpdate(g_kfSpeed, speed_obd_kmh_raw);
  float speed_gps_kmh = kalmanUpdate(g_kfGpsSpeed, speed_gps_kmh_raw);
  float gpsWeight = gpsLock ? (1.0f - (gpsHdop - 0.8f) / 2.0f) : 0.0f;
  if (gpsWeight < 0.0f) gpsWeight = 0.0f;
  if (gpsWeight > 0.85f) gpsWeight = 0.85f;
  float speed_kmh = (1.0f - gpsWeight) * speed_obd_kmh + gpsWeight * speed_gps_kmh;
  float rpm = kalmanUpdate(g_kfRpm, rpm_raw);
  float throttlePct = kalmanUpdate(g_kfThrottle, throttleRaw);
  float fuelRateLph = kalmanUpdate(g_kfFuelRate, fuelRateRaw);
  if (kUseDummyObd) {
    if (fuelRateLph > 38.0f) fuelRateLph = 38.0f;
    if (fuelRateLph < 0.9f) fuelRateLph = 0.9f;
  }
  float noiseRpm = fabsf(rpm_raw - rpm) / (rpm + 1.0f);
  float noiseSpeed = fabsf(speed_obd_kmh_raw - speed_obd_kmh) / (speed_obd_kmh + 1.0f);
  float noiseThrottle = fabsf(throttleRaw - throttlePct) / 100.0f;
  float noiseFuel = fabsf(fuelRateRaw - fuelRateLph) / (fuelRateLph + 1.0f);
  float noiseMix = 0.35f * noiseRpm + 0.25f * noiseSpeed + 0.20f * noiseThrottle + 0.20f * noiseFuel;
  g_signalNoisePct = 100.0f * noiseMix;
  if (g_signalNoisePct < 0.0f) g_signalNoisePct = 0.0f;
  if (g_signalNoisePct > 100.0f) g_signalNoisePct = 100.0f;

  // Broad torque hill ~95–315 Nm (typical turbo/strong NA for UI dummy).
  float torque = 200.0f + 115.0f * (1.0f - (rpm - 2600.0f) * (rpm - 2600.0f) / (2600.0f * 2600.0f));
  if (torque < 95.0f) torque = 95.0f;
  // Add randomness to simulate different vehicles/conditions
  torque *= (0.85f + 0.3f * sinf((float)millis() / 10000.0f));

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
  const float dtS = (float)g_wsPeriodMs / 1000.0f; // match actual WS broadcast period
  float autoSlopePct = 0.0f;
  if (fabsf(speedMps) > 1.0f) {
    autoSlopePct = ((gpsAltM - gpsAltPrevM) / (speedMps * dtS)) * 100.0f;
    if (autoSlopePct > 18.0f) autoSlopePct = 18.0f;
    if (autoSlopePct < -18.0f) autoSlopePct = -18.0f;
  }
  gpsAltPrevM = gpsAltM;
  float effectiveSlopePct = 0.65f * g_roadGradePct + 0.35f * autoSlopePct;
  static float prevSpeedMps = 0.0f;
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
  // Advanced wind estimate (dummy for now; replace with real GNSS/IMU model later).
  float windKmh = 8.0f * sinf(phase * 6.28318f); // head/tail wind estimate
  if (windKmh > 50.0f) windKmh = 50.0f;
  if (windKmh < -50.0f) windKmh = -50.0f;
  float vAirMps = speedMps + (windKmh / 3.6f);
  if (vAirMps < 0.0f) vAirMps = 0.0f;
  float fAeroN = 0.5f * rho * g_dragCd * g_frontalAreaM2 * vAirMps * vAirMps;
  float pAeroW = fAeroN * speedMps;
  float fRollN = g_rollResCoeff * (g_weightKg * 9.81f);
  float pRollW = fRollN * speedMps;
  float hpAeroLoss = pAeroW / 745.7f;
  float hpRollLoss = pRollW / 745.7f;

  // Slope correction (proxy until IMU+GPS fusion is integrated): user road grade input.
  float slopeRad = atanf(effectiveSlopePct / 100.0f);
  float fSlopeN = g_weightKg * 9.81f * sinf(slopeRad);
  float pSlopeW = fSlopeN * speedMps;
  float hpSlopeLoss = pSlopeW / 745.7f;

  // Pro physics path: engine force = m*a + drag + rolling + slope.
  float accelForEngine = accelMps2;
  if (kUseDummyObd) {
    if (accelForEngine > 6.5f) accelForEngine = 6.5f;
    if (accelForEngine < -5.5f) accelForEngine = -5.5f;
  }
  float fNetN = g_weightKg * accelForEngine;
  float fEngineN = fNetN + fAeroN + fRollN + fSlopeN;
  if (fEngineN < 0.0f) fEngineN = 0.0f;
  float torqueFromForceNm = fEngineN * g_wheelRadiusM;
  float omegaRadS = rpm * (2.0f * PI / 60.0f);
  float hpFromForce = (torqueFromForceNm * omegaRadS) / 745.7f;
  if (hpFromForce > 5.0f) {
    float hfBlend = hpFromForce;
    if (kUseDummyObd) {
      float hpFromTorqueOnly = torque * rpm / 7127.0f;
      if (hpFromTorqueOnly < 18.0f) hpFromTorqueOnly = 18.0f;
      float ceiling = hpFromTorqueOnly * 1.75f;
      if (ceiling > 260.0f) ceiling = 260.0f;
      if (hfBlend > ceiling) hfBlend = ceiling;
    }
    hpCrank = 0.5f * hpCrank + 0.5f * hfBlend;
  }

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

  // Self-calibration from collected runtime samples:
  // align physics-derived HP with measured crank HP while keeping bounded drift.
  if (g_selfCalEnabled && !g_selfCalLocked && throttlePct > 45.0f && rpm > 1800.0f && speed_kmh > 25.0f) {
    float err = hpFromForce - hpCrank;
    float adapt = 0.0006f * err; // conservative adaptation gain
    g_rollResCoeff += adapt * 0.02f;
    g_dragCd += adapt * 0.01f;
    g_wheelRadiusM += adapt * 0.0005f;
    if (g_rollResCoeff < 0.008f) g_rollResCoeff = 0.008f;
    if (g_rollResCoeff > 0.030f) g_rollResCoeff = 0.030f;
    if (g_dragCd < 0.20f) g_dragCd = 0.20f;
    if (g_dragCd > 0.55f) g_dragCd = 0.55f;
    if (g_wheelRadiusM < 0.24f) g_wheelRadiusM = 0.24f;
    if (g_wheelRadiusM > 0.42f) g_wheelRadiusM = 0.42f;

    float quality = 100.0f - fabsf(err) * 1.8f;
    if (quality < 0.0f) quality = 0.0f;
    if (quality > 100.0f) quality = 100.0f;
    g_selfCalConfidence = 0.96f * g_selfCalConfidence + 0.04f * quality;
    if (g_selfCalConfidence > 93.0f) g_selfCalLocked = true;
  }

  // Drift and resolution quality indicators.
  float drift = fabsf(hpFromForce - hpCrank) / (hpCrank + 1.0f) * 100.0f;
  g_signalDriftPct = drift;
  if (g_signalDriftPct > 100.0f) g_signalDriftPct = 100.0f;
  float rpmStep = fabsf(rpm - rpm_raw);
  float res = 100.0f - (rpmStep * 0.04f + g_signalNoisePct * 0.4f);
  if (res < 0.0f) res = 0.0f;
  if (res > 100.0f) res = 100.0f;
  g_signalResolutionPct = 0.95f * g_signalResolutionPct + 0.05f * res;

  if (g_selfCalLocked && !g_selfCalSaved && !g_prefsSaveSelfCalPending) {
    g_prefsSaveSelfCalPending = true;
  }

  // Basic predictive/anomaly placeholder for online diagnostics.
  // Gear detection from RPM/speed ratio (advanced heuristic).
  float speedMpsSafe = speedMps < 0.5f ? 0.5f : speedMps;
  float wheelRps = speedMpsSafe / (2.0f * PI * g_wheelRadiusM);
  float engineRps = rpm / 60.0f;
  float overallRatio = engineRps / wheelRps;
  float gearDetected = 0.0f;
  if (overallRatio > 0.5f) gearDetected = overallRatio / (g_finalDriveRatio > 0.1f ? g_finalDriveRatio : 4.1f);

  // Integer gear guess (typical 6-speed map; can become user-configurable).
  const float kGearMap[6] = {3.60f, 2.10f, 1.45f, 1.10f, 0.92f, 0.78f};
  int gearDetectedInt = 0;
  float gearBestErr = 1e9f;
  for (int i = 0; i < 6; i++) {
    float e = fabsf(gearDetected - kGearMap[i]);
    if (e < gearBestErr) {
      gearBestErr = e;
      gearDetectedInt = i + 1;
    }
  }
  if (gearDetected < 0.5f) gearDetectedInt = 0;

  // Tire/RPM speed estimation used as validation path (GPS remains primary).
  float usedGearRatio = g_gearRatio > 0.2f ? g_gearRatio : (gearDetected > 0.2f ? gearDetected : 3.5f);
  float wheelRpsFromRpm = engineRps / ((g_finalDriveRatio > 0.1f ? g_finalDriveRatio : 4.1f) * usedGearRatio);
  float speedRpmTireKmh = wheelRpsFromRpm * (2.0f * PI * g_wheelRadiusM) * 3.6f;
  if (speedRpmTireKmh < 0.0f) speedRpmTireKmh = 0.0f;
  // Dummy GPS loop path for track-mode lap geofence detection.
  const float trackPhase = (t_ms % 90000) / 90000.0f;
  const float ang = trackPhase * 2.0f * PI;
  const float latCenter = 41.9965f;
  const float lonCenter = 21.4314f;
  const float latAmp = 0.00045f;
  const float lonAmp = 0.00058f;
  const float gpsLat = latCenter + latAmp * cosf(ang);
  const float gpsLon = lonCenter + lonAmp * sinf(ang);
  float slipPct = 0.0f;
  if (speed_gps_kmh > 5.0f) {
    slipPct = fabsf(speedRpmTireKmh - speed_gps_kmh) / speed_gps_kmh * 100.0f;
    if (slipPct > 80.0f) slipPct = 80.0f;
  }

  // Expected power now uses detected gear factor (intelligent feature in calculations).
  float gearFactor = (gearDetectedInt > 0) ? (1.0f + 0.03f * (float)gearDetectedInt) : 1.0f;
  float hpExpected = (throttlePct / 100.0f) * (0.065f * rpm) * gearFactor;
  const char* anomaly = "none";
  if (throttlePct > 65.0f && hpCrank < hpExpected * 0.75f) anomaly = "power_drop";
  if (throttlePct > 55.0f && fuelRateLph < 3.0f) anomaly = "fuel_rate_low";

  // Coast-down calibration validity gate (used for PRO guidance in UI).
  g_coastCalConfidence = g_selfCalConfidence;
  g_coastCalValid = g_selfCalLocked && g_selfCalConfidence >= 70.0f
    && g_signalNoisePct <= 20.0f && g_signalDriftPct <= 20.0f;
  if (g_coastCalValid) g_coastCalReason = "ok";
  else if (!g_selfCalLocked) g_coastCalReason = "calibration not locked";
  else if (g_signalNoisePct > 20.0f) g_coastCalReason = "signal noise too high";
  else if (g_signalDriftPct > 20.0f) g_coastCalReason = "signal drift too high";
  else if (g_selfCalConfidence < 70.0f) g_coastCalReason = "confidence too low";
  else g_coastCalReason = "repeat procedure";

  float batteryV = readBatteryVoltageV();
  float batteryPct = NAN;
  const char* batteryState = "unknown";
  const char* batteryProfile = "unknown";

  if (kBatteryDemoPayload) {
    // Demo: cycle pack SOC / color states every 10 s (kBatteryDemoPayload).
    unsigned step = (t_ms / 10000u) % 3u;
    batteryProfile = "li2s";
    if (step == 0u) batteryV = 8.05f;
    else if (step == 1u) batteryV = 7.05f;
    else batteryV = 6.18f;
    batteryPct = ((batteryV - kBatteryLi2sMinV) / (kBatteryLi2sMaxV - kBatteryLi2sMinV)) * 100.0f;
    if (batteryPct < 0.0f) batteryPct = 0.0f;
    if (batteryPct > 100.0f) batteryPct = 100.0f;
  } else if (!isnan(batteryV)) {
    if (batteryV <= 9.5f) {
      batteryProfile = "li2s";
      batteryPct = ((batteryV - kBatteryLi2sMinV) / (kBatteryLi2sMaxV - kBatteryLi2sMinV)) * 100.0f;
      if (batteryPct < 0.0f) batteryPct = 0.0f;
      if (batteryPct > 100.0f) batteryPct = 100.0f;
    } else {
      batteryProfile = "external";
      batteryPct = -1.0f;
      batteryState = "unknown";
    }
  }

  if (!isnan(batteryPct) && batteryPct >= 0.0f) {
    if (batteryPct >= 78.0f) batteryState = "green";
    else if (batteryPct >= 15.0f) batteryState = "yellow";
    else batteryState = "red";
  }

  float batteryVJson = isnan(batteryV) ? 0.0f : batteryV;
  float batteryPctJson = isnan(batteryPct) ? -1.0f : batteryPct;

  int n = snprintf(g_wsLiveJsonBuf, sizeof(g_wsLiveJsonBuf),
           "{\"type\":\"live\",\"t_ms\":%lu,\"speed_kmh\":%.1f,\"speed_obd_kmh\":%.1f,\"speed_gps_kmh\":%.1f,\"speed_fused_kmh\":%.1f,\"speed_rpm_tire_kmh\":%.1f,\"speed_gps_weight\":%.2f,\"slip_pct\":%.1f,\"accel_mps2\":%.3f,\"rpm\":%.0f,\"hp\":%.0f,\"torque_nm\":%.0f,\"hp_wheel\":%.1f,\"hp_crank\":%.1f,\"hp_corrected\":%.1f,\"corr_factor_k\":%.3f,\"corr_std\":\"%s\",\"hp_expected\":%.1f,\"anomaly\":\"%s\",\"hp_loss_total\":%.1f,\"hp_loss_aero\":%.1f,\"hp_loss_roll\":%.1f,\"hp_loss_slope\":%.1f,\"loss_gearbox_pct\":%.1f,\"drive_type\":\"%s\",\"throttle_pct\":%.1f,\"fuel_rate_lph\":%.2f,\"air_intake_c\":%.1f,\"engine_oil_c\":%.1f,\"air_density\":%.4f,\"pressure_hpa\":%.1f,\"humidity_pct\":%.1f,\"gps_lock\":%s,\"gps_sats\":%d,\"gps_hdop\":%.2f,\"gnss_mode\":\"%s\",\"gps_lat\":%.7f,\"gps_lon\":%.7f,\"redline_rpm\":%.0f,\"power_unit\":\"%s\",\"auto_slope_pct\":%.2f,\"wind_kmh\":%.1f,\"gear_detected\":%.2f,\"gear_detected_int\":%d,\"self_cal_enabled\":%s,\"self_cal_locked\":%s,\"self_cal_conf\":%.1f,\"coast_bypass\":%s,\"coast_cal_valid\":%s,\"coast_cal_conf\":%.1f,\"coast_cal_reason\":\"%s\",\"signal_noise_pct\":%.1f,\"signal_drift_pct\":%.1f,\"signal_resolution_pct\":%.1f,\"battery_v\":%.3f,\"battery_pct\":%.1f,\"battery_state\":\"%s\",\"battery_profile\":\"%s\",\"auto_arm_kmh\":%.1f,\"measurement_auto_arm\":%s,\"ap_clients\":%d,\"setup_ok\":%s,\"missing_fields\":%s}",
           (unsigned long)t_ms, speed_kmh, speed_obd_kmh, speed_gps_kmh, speed_kmh, speedRpmTireKmh, gpsWeight, slipPct, accelMps2, rpm, hp, torque, hpWheel, hpCrank, hpCorrected, corrK, g_corrStandard.c_str(), hpExpected, anomaly, hpLossTotal, hpAeroLoss, hpRollLoss, hpSlopeLoss, g_gearboxLossPct, g_driveType.c_str(), throttlePct, fuelRateLph, airIntakeC, engineOilC, rho, pressurePa / 100.0f, relHum, gpsLock ? "true" : "false", gpsSats, gpsHdop, gnssMode, gpsLat, gpsLon, g_redlineRpm, g_powerUnitPref.c_str(), autoSlopePct, windKmh, gearDetected, gearDetectedInt, g_selfCalEnabled ? "true" : "false", g_selfCalLocked ? "true" : "false", g_selfCalConfidence,
           g_coastBypass ? "true" : "false", g_coastCalValid ? "true" : "false", g_coastCalConfidence, g_coastCalReason.c_str(), g_signalNoisePct, g_signalDriftPct, g_signalResolutionPct, batteryVJson, batteryPctJson, batteryState, batteryProfile, g_autoArmSpeedKmh, g_measurementAutoArm ? "true" : "false", WiFi.softAPgetStationNum(),
           g_setupOk ? "true" : "false",
           g_missingFieldsJson.c_str());
  if (n <= 0 || n >= (int)sizeof(g_wsLiveJsonBuf)) {
    static unsigned long lastLogMs = 0;
    unsigned long m = millis();
    if (m - lastLogMs > 3000) {
      lastLogMs = m;
      Serial.printf("WS JSON truncated or fmt err (n=%d, cap=%u)\n", n, (unsigned)sizeof(g_wsLiveJsonBuf));
    }
    return 0;
  }
  return n;
}

static void onWsLiveEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
                          void* arg, uint8_t* data, size_t len) {
  (void)server;
  (void)arg;
  (void)data;
  (void)len;
  if (type == WS_EVT_CONNECT) {
    // Library default closeWhenFull=true drops the TCP connection when the send queue fills
    // (common on phone WiFi to the ESP AP). That looks like a stuck "OBDII: connecting…" UI.
    client->setCloseClientOnQueueFull(false);
    // Periodic WS PING when outbound queues are idle — helps TCP/NAT stay warm (library default is 0 = off).
    client->keepAlivePeriod(20);
    Serial.printf("[WS] client %u connected (heap=%lu)\n", client->id(), (unsigned long)ESP.getFreeHeap());
    const int n = formatLiveJson((uint32_t)millis());
    if (n > 0) {
      client->text(g_wsLiveJsonBuf, (size_t)n);
    }
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] client %u disconnected\n", client->id());
  } else if (type == WS_EVT_ERROR) {
    Serial.printf("[WS] client %u error\n", client->id());
  }
}

static bool hasFormField(AsyncWebServerRequest* r, const char* name) {
  return r->hasParam(name, true);
}

static String formField(AsyncWebServerRequest* r, const char* name) {
  if (!r->hasParam(name, true)) return String();
  const AsyncWebParameter* p = r->getParam(name, true);
  return p ? p->value() : String();
}

static bool startAccessPoint() {
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gw(192, 168, 4, 1);
  IPAddress nm(255, 255, 255, 0);

  const int kApChannel = 1;       // widely compatible with phones
  const bool kApHidden = false;
  const int kApMaxConnections = 6;

  // Clear stale state (helps after flash / OTA / rare radio stuck states).
  WiFi.disconnect(true);
  delay(80);

  // Some ESP32 Arduino / IDF builds expect config before softAP; if AP IP stays 0.0.0.0, retry order.
  WiFi.softAPConfig(apIP, gw, nm);
  delay(50);

  bool ok = WiFi.softAP(kApSsid, nullptr, kApChannel, kApHidden, kApMaxConnections);
  if (!ok) {
    Serial.println("softAP failed (1), retrying...");
    delay(200);
    ok = WiFi.softAP(kApSsid, nullptr, kApChannel, kApHidden, kApMaxConnections);
  }
  if (!ok) {
    Serial.println("Failed to start WiFi AP");
    return false;
  }

  delay(120);
  // Ensure subnet config applied (call again after AP is up on some cores).
  WiFi.softAPConfig(apIP, gw, nm);

  IPAddress sip = WiFi.softAPIP();
  if (sip == IPAddress(0, 0, 0, 0)) {
    Serial.println("AP IP was 0.0.0.0, restarting softAP...");
    WiFi.softAPdisconnect(true);
    delay(150);
    ok = WiFi.softAP(kApSsid, nullptr, kApChannel, kApHidden, kApMaxConnections);
    delay(120);
    WiFi.softAPConfig(apIP, gw, nm);
    sip = WiFi.softAPIP();
  }

  Serial.print("AP SSID: ");
  Serial.println(kApSsid);
  Serial.print("AP IP: ");
  Serial.println(sip);
  // Higher TX power often stabilizes AP + TCP (fewer WS disconnects on phones).
  WiFi.setTxPower(WIFI_POWER_15dBm);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("DynoTrack X - skeleton starting...");
  if (kBatteryDemoPayload) {
    Serial.println("[Battery] Demo WebSocket values ON — set kBatteryDemoPayload=false after wiring ADC.");
  }

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(kUseDummyObd ? WIFI_AP : WIFI_AP_STA);
  WiFi.setSleep(false); // keep WiFi radio responsive for AP + WS traffic
  analogReadResolution(12);
  analogSetPinAttenuation(kBatterySensePin, ADC_11db);
  delay(100);           // let radio settle before softAP (fixes flaky AP on some S3 boards)

  if (kUseDummyObd) {
    Serial.println("OBDII DUMMY mode enabled. Skipping real OBDII WiFi connect.");
    startAccessPoint();
  } else {
    // Bring AP up first so UI/web server are reachable immediately.
    startAccessPoint();
    // Serial.print("Connecting to OBDII SSID: ");
    // Serial.println(kObdSsid);
    // WiFi.begin(kObdSsid);

    // const uint32_t kObdConnectTimeoutMs = 12000;
    // unsigned long connectStart = millis();
    // while (WiFi.status() != WL_CONNECTED && millis() - connectStart < kObdConnectTimeoutMs) {
    //   delay(250);
    //   Serial.print(".");
    // }
    // Serial.println();
    // if (WiFi.status() == WL_CONNECTED) {
    //   Serial.print("OBDII connected. STA IP: ");
    //   Serial.println(WiFi.localIP());
    // } else {
    //   Serial.println("OBDII WiFi connect timeout. Continuing with AP mode active.");
    // }
  }

  loadSettings();

  wsLive.onEvent(onWsLiveEvent);
  httpServer.addHandler(&wsLive);

  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    // PROGMEM HTML: must use progmem length + uint8_t* path → AsyncProgmemResponse (memcpy_P).
    // Plain send(..., const char*) uses AsyncBasicResponse and reads flash as RAM → blank page.
    AsyncWebServerResponse* res = request->beginResponse(200, "text/html", reinterpret_cast<const uint8_t*>(kHomeHtml), strlen_P(kHomeHtml));
    res->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    res->addHeader("Pragma", "no-cache");
    res->addHeader("Expires", "0");
    request->send(res);
  });
  httpServer.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* res = request->beginResponse(200, "text/html", reinterpret_cast<const uint8_t*>(kSettingsHtml), strlen_P(kSettingsHtml));
    res->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    res->addHeader("Pragma", "no-cache");
    res->addHeader("Expires", "0");
    request->send(res);
  });
  httpServer.on("/logo.png", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "image/png", reinterpret_cast<const uint8_t*>(kLogoPng), kLogoPngLen);
  });

  httpServer.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", jsonApiSettings());
  });

  httpServer.on("/health", HTTP_GET, [](AsyncWebServerRequest* request) {
    char buf[320];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"ap_ip\":\"%s\",\"ap_clients\":%d,\"ws_clients\":%u,\"wifi_mode\":%d,\"heap\":%lu}",
             WiFi.softAPIP().toString().c_str(),
             WiFi.softAPgetStationNum(),
             (unsigned)wsLive.count(),
             (int)WiFi.getMode(),
             (unsigned long)ESP.getFreeHeap());
    request->send(200, "application/json", String(buf));
  });

  httpServer.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest* request) {
    // Expect x-www-form-urlencoded params in body
    if (!hasFormField(request, "weightKg") || !hasFormField(request, "tireSize")) {
      request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing mandatory fields\"}");
      return;
    }

    g_weightKg = formField(request, "weightKg").toFloat();
    g_tireSize = formField(request, "tireSize");
    g_tireSize.trim();

    if (hasFormField(request, "humidityPct")) {
      String v = formField(request, "humidityPct");
      if (v.length() > 0) g_humidityPct = v.toFloat();
      else g_humidityPct = NAN;
    }

    if (hasFormField(request, "pressureHpa")) {
      String v = formField(request, "pressureHpa");
      if (v.length() > 0) g_pressureHpa = v.toFloat();
      else g_pressureHpa = NAN;
    }

    if (hasFormField(request, "ambientTempNoteC")) {
      String v = formField(request, "ambientTempNoteC");
      v.trim();
      if (v.length() > 0) {
        g_ambientTempNoteC = v.toFloat();
        if (g_ambientTempNoteC < -80.0f) g_ambientTempNoteC = -80.0f;
        if (g_ambientTempNoteC > 80.0f) g_ambientTempNoteC = 80.0f;
      } else {
        g_ambientTempNoteC = NAN;
      }
    }

    if (hasFormField(request, "unitsMetric")) {
      g_unitsMetric = (formField(request, "unitsMetric") == "1");
    }

    if (hasFormField(request, "finalDriveRatio") && formField(request, "finalDriveRatio").length() > 0) {
      g_finalDriveRatio = formField(request, "finalDriveRatio").toFloat();
    }
    if (hasFormField(request, "gearRatio") && formField(request, "gearRatio").length() > 0) {
      g_gearRatio = formField(request, "gearRatio").toFloat();
    }
    if (hasFormField(request, "drivetrainLossPct") && formField(request, "drivetrainLossPct").length() > 0) {
      g_gearboxLossPct = formField(request, "drivetrainLossPct").toFloat();
    }
    if (hasFormField(request, "driveType") && formField(request, "driveType").length() > 0) {
      g_driveType = formField(request, "driveType");
      g_driveType.toLowerCase();
      if (g_driveType != "fwd" && g_driveType != "rwd" && g_driveType != "awd") g_driveType = "fwd";
    }
    if (hasFormField(request, "dragCd") && formField(request, "dragCd").length() > 0) {
      g_dragCd = formField(request, "dragCd").toFloat();
    }
    if (hasFormField(request, "frontalAreaM2") && formField(request, "frontalAreaM2").length() > 0) {
      g_frontalAreaM2 = formField(request, "frontalAreaM2").toFloat();
    }
    if (hasFormField(request, "rollResCoeff") && formField(request, "rollResCoeff").length() > 0) {
      g_rollResCoeff = formField(request, "rollResCoeff").toFloat();
    }
    if (hasFormField(request, "roadGradePct") && formField(request, "roadGradePct").length() > 0) {
      g_roadGradePct = formField(request, "roadGradePct").toFloat();
    }
    if (hasFormField(request, "wheelRadiusM") && formField(request, "wheelRadiusM").length() > 0) {
      g_wheelRadiusM = formField(request, "wheelRadiusM").toFloat();
    }
    if (hasFormField(request, "corrStandard") && formField(request, "corrStandard").length() > 0) {
      g_corrStandard = formField(request, "corrStandard");
      g_corrStandard.toLowerCase();
      if (g_corrStandard != "din" && g_corrStandard != "sae") g_corrStandard = "din";
    }
    if (hasFormField(request, "powerUnit") && formField(request, "powerUnit").length() > 0) {
      g_powerUnitPref = formField(request, "powerUnit");
      g_powerUnitPref.toLowerCase();
      if (g_powerUnitPref != "hp" && g_powerUnitPref != "kw") g_powerUnitPref = "hp";
    }
    if (hasFormField(request, "redlineRpm") && formField(request, "redlineRpm").length() > 0) {
      g_redlineRpm = formField(request, "redlineRpm").toFloat();
      if (g_redlineRpm < 3000.0f) g_redlineRpm = 3000.0f;
      if (g_redlineRpm > 11000.0f) g_redlineRpm = 11000.0f;
    }
    if (hasFormField(request, "coastBypass")) {
      g_coastBypass = (formField(request, "coastBypass") == "0");
    }

    if (hasFormField(request, "autoArmKmh") && formField(request, "autoArmKmh").length() > 0) {
      g_autoArmSpeedKmh = formField(request, "autoArmKmh").toFloat();
      if (g_autoArmSpeedKmh < 0.0f) g_autoArmSpeedKmh = 0.0f;
      if (g_autoArmSpeedKmh > 200.0f) g_autoArmSpeedKmh = 200.0f;
    }
    if (hasFormField(request, "measurementAutoArm")) {
      g_measurementAutoArm = (formField(request, "measurementAutoArm") == "1");
    }

    if (!parseMandatoryFields()) {
      recomputeSetupState();
      request->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid mandatory fields\"}");
      return;
    }

    prefs.begin("dyntx", false);
    prefs.putFloat("weightKg", g_weightKg);
    prefs.putString("tireSize", g_tireSize);
    if (!isnan(g_humidityPct)) prefs.putFloat("humidityPct", g_humidityPct);
    if (!isnan(g_pressureHpa)) prefs.putFloat("pressureHpa", g_pressureHpa);
    if (!isnan(g_ambientTempNoteC))
      prefs.putFloat("ambNoteC", g_ambientTempNoteC);
    else
      prefs.remove("ambNoteC");
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
    prefs.putString("powerUnit", g_powerUnitPref);
    prefs.putFloat("redlineRpm", g_redlineRpm);
    prefs.putBool("coastBypass", g_coastBypass);
    prefs.putFloat("autoArmKmh", g_autoArmSpeedKmh);
    prefs.putBool("measAutoArm", g_measurementAutoArm);
    prefs.putBool("coastValid", false);
    prefs.putFloat("coastConf", 0.0f);
    prefs.putString("coastReason", "settings changed");
    prefs.end();

    g_coastCalValid = false;
    g_coastCalConfidence = 0.0f;
    g_coastCalReason = "settings changed";

    recomputeSetupState();

    request->send(200, "application/json", "{\"ok\":true}");
  });

  httpServer.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });
  httpServer.begin();

  Serial.println("HTTP + WebSocket (/ws) on port 80 started.");
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  wsLive.cleanupClients();

  /* Extra WebSocket ping to all clients — backup keepalive during long sessions (phones may sleep radio). */
  static unsigned long s_lastWsPingAllMs = 0;
  const unsigned long pingNow = millis();
  if (wsLive.count() > 0 && (pingNow - s_lastWsPingAllMs) >= 25000) {
    s_lastWsPingAllMs = pingNow;
    wsLive.pingAll();
  }

  if (g_prefsSaveSelfCalPending) {
    g_prefsSaveSelfCalPending = false;
    prefs.begin("dyntx", false);
    prefs.putFloat("dragCd", g_dragCd);
    prefs.putFloat("rollRes", g_rollResCoeff);
    prefs.putFloat("wheelRadM", g_wheelRadiusM);
    prefs.putBool("coastValid", g_coastCalValid);
    prefs.putFloat("coastConf", g_coastCalConfidence);
    prefs.putString("coastReason", g_coastCalReason);
    prefs.end();
    g_selfCalSaved = true;
  }

  // WiFi self-healing: if AP drops, restart it.
  static unsigned long lastWifiCheckMs = 0;
  const unsigned long nowCheck = millis();
  if (nowCheck - lastWifiCheckMs >= 3000) {
    lastWifiCheckMs = nowCheck;
    static uint8_t apUnhealthyCount = 0;
    static unsigned long lastApRestartMs = 0;
    const bool apHealthy = (WiFi.softAPIP() != IPAddress(0, 0, 0, 0));
    if (apHealthy) {
      apUnhealthyCount = 0;
    } else {
      apUnhealthyCount++;
      const bool cooldownOk = (nowCheck - lastApRestartMs) > 15000;
      if (apUnhealthyCount >= 3 && cooldownOk) {
        Serial.println("AP unhealthy for multiple checks, restarting AP...");
        startAccessPoint();
        lastApRestartMs = nowCheck;
        apUnhealthyCount = 0;
      }
    }
  }

  static bool schedulerInit = false;
  static unsigned long nextTickMs = 0;
  static uint32_t tMs = 0;
  const uint32_t periodMs = g_wsPeriodMs;

  unsigned long nowMs = millis();
  if (!schedulerInit) {
    schedulerInit = true;
    nextTickMs = nowMs + periodMs;
  }

  // At most one dummy WS payload per loop(); resync if we fell far behind (no tight catch-up loop).
  if ((int32_t)(nowMs - nextTickMs) >= 0) {
    nextTickMs += periodMs;
    tMs += periodMs;
    const int32_t backlog = (int32_t)(nowMs - nextTickMs);
    if (backlog > (int32_t)(6 * (int32_t)periodMs)) {
      nextTickMs = nowMs + periodMs;
    }
    if (wsLive.count() > 0) {
      const int n = formatLiveJson(tMs);
      if (n > 0) {
        wsLive.textAll(g_wsLiveJsonBuf, (size_t)n);
      }
    }
  }

  esp_task_wdt_reset();
  yield();
}