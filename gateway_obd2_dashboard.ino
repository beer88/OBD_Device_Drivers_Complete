#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_heap_caps.h>
#if defined(ARDUINO_ARCH_ESP32)
  #include <esp32/spiram.h>
#endif
#include <memory>
#include "driver/twai.h"
#include <Preferences.h>

// ===================== User Config =====================
// AP credentials
static const char* WIFI_AP_SSID = "Wireless-OBDII";
static const char* WIFI_AP_PASS = "mrdiy.ca"; 

// CAN shield pin mapping (adjust if needed)
#define SHIELD_CAN_RX GPIO_NUM_4
#define SHIELD_CAN_TX GPIO_NUM_5

#define SHIELD_LED_GREEN_PIN  26
#define SHIELD_LED_BLUE_PIN  27


// Queue and buffer sizing
static const uint32_t CAN_RX_TASK_STACK = 4096;
// static const uint32_t FLUSH_TASK_STACK = 4096; // no flush task in polling mode
static const uint32_t CAN_RX_QUEUE_LEN = 64;   // TWAI driver RX queue
static const uint32_t CAN_TX_QUEUE_LEN = 32;   // TWAI driver TX queue

// Aggregator table sizing
static const size_t AGG_TABLE_SIZE = 2048;     // reduce DRAM; power of two preferred but not required
static const uint32_t FLUSH_INTERVAL_MS = 50;  // how often to push UI updates
static const size_t MAX_WS_CLIENTS = 4;
static volatile bool g_can_running = false;
static uint32_t g_current_bitrate = 500000;   // default 500 Kbps
static volatile uint32_t g_last_can_rx_ms = 0; // for LED activity
static volatile uint32_t g_tx_count = 0; // TX frame counter
static volatile uint32_t g_rx_count = 0; // RX frame counter
static volatile uint32_t g_rx_fps = 0; // RX frames per second
static volatile uint32_t g_tx_fps = 0; // TX frames per second
static uint32_t g_last_stats_ms = 0; // Last stats calculation time
static volatile bool g_can_traffic_detected = false; // CAN traffic indicator
static volatile bool g_obd_responses_detected = false; // OBD responses indicator
static volatile uint32_t g_last_can_traffic_ms = 0; // Last CAN traffic time
static volatile uint32_t g_last_obd_response_ms = 0; // Last OBD response time
static Preferences g_prefs;
// OBD TX control and fault tracking
static volatile bool g_obd_tx_enabled = false; // whether we actively send 0x7DF requests
static volatile uint8_t g_tx_failures = 0; // consecutive transmit failures
static volatile uint8_t g_tx_fail_streak = 0; // streak used to detect blocked writes
static volatile bool g_tx_write_blocked = false; // likely transceiver listen-only / TX inhibited
static volatile bool g_tx_suppressed = false; // stop any further TX attempts to protect RX stability

// ===================== OBD-II State =====================
struct ObdState {
  bool     enabled;
  uint32_t lastPollMs;
  // PIDs
  float    engineLoadPct;
  float    rpm;
  float    speedKph;
  float    fuelPct;
  float    throttlePct;
  int16_t  coolantC; // Celsius
  float    intakeMapKpa;
  float    intakeAirTempC;
  float    mafGps;
  float    timingAdvanceDeg;
  float    baroKpa;
  float    ambientAirTempC;
  uint32_t runtimeSec;
  float    evapPurgePct;
  float    moduleVoltageV; // Control module (battery/charging) voltage
  // Mode 01 PID 0x1C
  uint8_t  obdStdCode;      // numeric code
  // Mode 09 VIN and ECU name (assembled from multi-frames of 49 02 and 49 0A)
  char     vin[18];         // 17 chars + NUL
  uint8_t  vinLen;
  char     ecuName[33];     // up to 32 chars + NUL
  uint8_t  ecuLen;
  // Mode 01 PID 00 supported bitmap (PIDs 0x01..0x20) A..D
  uint32_t pid00Mask;
  bool     pid00Valid;
  // DTCs (first few)
  char     dtc[5][8]; // up to 5 codes, 7-char strings like P0301
  uint8_t  dtcCount;
};
static ObdState g_obd;
static SemaphoreHandle_t g_obdMutex;
static const uint8_t OBD_PIDS[] = {
  0x00, // Supported PIDs 01-20
  0x04, // Calculated engine load
  0x05, // Coolant temp
  0x0B, // Intake manifold absolute pressure
  0x0C, // RPM
  0x0D, // Speed
  0x0E, // Timing advance
  0x0F, // Intake air temp
  0x10, // MAF air flow rate
  0x11, // Throttle position
  0x2F, // Fuel level
  0x1F, // Runtime since engine start (seconds)
  0x31, // Distance traveled with MIL on (runtime)
  0x4D, // Time run with MIL on (runtime in minutes)
  0x42, // Control module voltage (battery/charging voltage)
  0x5B, // Hybrid battery pack remaining life (if supported)
  0x33, // Barometric pressure
  0x46, // Ambient air temp
  0x4D, // Time run with MIL on (optional)
  0x52, // EVAP purge (commanded)
  0x1C, // OBD standards this vehicle conforms to
};
static const size_t NUM_OBD_PIDS = sizeof(OBD_PIDS);
static uint32_t g_obd_poll_interval_ms = 100; // send one PID every 100 ms

static inline void send_obd_pid_request(uint8_t pid) {
  if (g_tx_suppressed || g_tx_write_blocked) return; // gracefully skip TX in RO mode
  twai_message_t m = {};
  m.identifier = 0x7DF; // functional request
  m.extd = 0;
  m.rtr = 0;
  m.data_length_code = 8;
  m.data[0] = 0x02; // single frame, 2 data bytes follow
  m.data[1] = 0x01; // service 01 - current data
  m.data[2] = pid;
  for (int i = 3; i < 8; ++i) m.data[i] = 0x00;
  esp_err_t txr = twai_transmit(&m, pdMS_TO_TICKS(10));
  if (txr == ESP_OK) {
    g_tx_count++;
    g_tx_fail_streak = 0;
  } else {
    if (g_tx_fail_streak < 255) g_tx_fail_streak++;
    if (g_tx_fail_streak >= 3) { g_tx_write_blocked = true; g_tx_suppressed = true; }
  }
}

static inline void send_obd_mode09_request(uint8_t pid) {
  if (g_tx_suppressed || g_tx_write_blocked) return;
  twai_message_t m = {};
  m.identifier = 0x7DF;
  m.extd = 0;
  m.rtr = 0;
  m.data_length_code = 8;
  m.data[0] = 0x02; // two following bytes
  m.data[1] = 0x09; // service 09
  m.data[2] = pid;  // PID (02 VIN, 0A ECU name)
  for (int i = 3; i < 8; ++i) m.data[i] = 0x00;
  esp_err_t txr = twai_transmit(&m, pdMS_TO_TICKS(10));
  if (txr == ESP_OK) { g_tx_count++; g_tx_fail_streak = 0; }
  else { if (g_tx_fail_streak < 255) g_tx_fail_streak++; if (g_tx_fail_streak >= 3) { g_tx_write_blocked = true; g_tx_suppressed = true; } }
}



// ===================== OBD UI (inline) =====================
static const char OBD_HTML[] PROGMEM = R"_O(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <link rel="prefetch" href="/msg" />
  <link rel="icon" type="image/png" sizes="16x16" href='data:image/x-icon;base64,AAABAAEAEBAAAAEAIABoBAAAFgAAACgAAAAQAAAAIAAAAAEAIAAAAAAAQAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAIdUOc6IVTvpiFU654hVOuaIVTvkiVU74IdUOsqFUDaMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACIVDvli1c9/4pWPf+KVj3/ilY9/4pXPf+LVz3/jVk+/4lWPPaBSS5pAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgk064oVRPf+EUDz/hFA8/4RQPP98Rz3/fUk8/4VRPP+KVzz/jVk+/4ROMXwAAAAA/8s1///LNf//yzX//8s1///KM///yjL//8oy///KMv//yjL//8ky//C4Nf+wfjr/f0o8/4pWPP+LVz3/AAAAAP/FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xjP//8wy/76LOf+DTzz/jFg9/4ZTOLX/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xzP/f0s8/4pWPP+IVTvj/8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8oy/6JwO/+HUzz/ilY88//FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///KMv+fbTv/h1Q8/4pWPPP/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///GM///xTP/fUg8/4pWPP+JVjri/8Yz///GM///xjP//8Yz///GM///xjP//8Yz///GM///xjP//8Yz///HM///yzL/sX46/4RRPP+MWD3/h1I4sf/GMvT/xjL0/8Yy9P/FMvP4wDP99740//e+NP/3vjT/9740//O6NP/dpzf/m2k7/4JOPP+KVjz/iVY8/wAAAAAAAAAAAAAAAAAAAAAAAAAAd0A74X5IPf98Rzz/fEc8/3xHPP99SDz/gEw8/4dTPP+LVz3/jFg9/4FKL2sAAAAAAAAAAAAAAAAAAAAAAAAAAIhVOuaMWD7/i1c9/4tXPf+LVz3/i1c9/4xYPf+OWT//iFQ75XxDJ0gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACHUji2iFQ6zohTOc2HVDnMh1M5yodUOsaGUjerf0YqVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA//8AAPAPAADwAwAA8AEAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAA8AEAAPADAADwDwAA//8AAA=='/>
  <title>MrDIY - Live CAN with OBD-II</title>
  <style>
    :root{--brand:#32c5ff;--ink:#123;--muted:#567;--bg:#f7f9fb}
    *{box-sizing:border-box}
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:var(--bg);color:var(--ink)}
    header{background:#ffffff;color:var(--brand);border-bottom: 2px solid var(--brand);padding:10px 16px;margin-bottom:10px;display:flex;align-items:center;gap:16px}    a{color:#fff;text-decoration:underline}
    h3{margin: 0 0 15px 0;}
    .toplink{color:var(--brand);margin-left:auto;text-decoration:none}
    main{padding:14px; max-width:1024px; margin:0 auto}
    .layout{display:block}
    @media (max-width: 900px){.layout{grid-template-columns:1fr}}
    .panel{background:#fff;border-radius:12px;box-shadow:0 1px 6px rgba(0,0,0,.06);padding:12px}
    .label{font-size:12px;color:var(--muted)}
    .value{font-size:28px;font-weight:700}
          .unit{font-size:14px;color:var(--muted);margin-left:4px}
      .grid{display:grid;grid-template-columns: repeat(auto-fit,minmax(180px,1fr)); gap:12px}
      .bar{height:10px;background:#eee;border-radius:8px;overflow:hidden;margin: 10px 0;}
      .bar>span{display:block;height:100%;background:var(--brand)}
      .stack{display:flex;flex-direction:column;gap:10px}
      
      /* Mobile responsive gauges */
      @media (max-width: 768px) {
        .gauge-container { width: 120px !important; height: 120px !important; }
        .gauge-container canvas { width: 120px !important; height: 120px !important; }
        .gauge-value { font-size: 18px !important; }
        .gauge-label { font-size: 12px !important; }
      }
    .dtc{display:flex;flex-wrap:wrap;gap:6px;margin-top:6px}
    .pill{background:#f0f7ff;border:1px solid #bfe6ff;color:#036;margin:0 0 0 40px;padding:3px 8px;border-radius:999px;font-size:12px}
    canvas{display:block;width:100%;height:auto}
    .subgrid{display:grid;grid-template-columns: repeat(auto-fit,minmax(140px,1fr)); gap:10px}
    .muted{color:var(--muted);font-size:12px}
    #logo{ padding-left:55px;background-size: 37px 16px;height: 17px; background-repeat: no-repeat;background-image: url("data:image/svg+xml,%3Csvg width='469' height='197' viewBox='0 0 469 197' fill='none' xmlns='http://www.w3.org/2000/svg'%3E%3Crect width='469' height='197' fill='%23F2F2F2'/%3E%3Crect width='469' height='197' fill='white'/%3E%3Cpath d='M127.125 196C158.396 196 184.026 186.794 204.016 168.382C224.005 149.97 234 126.212 234 97.1091C234 68.0061 224.104 44.5455 204.312 26.7273C184.521 8.90909 158.792 0 127.125 0H63V196H127.125Z' fill='%233D568A'/%3E%3Cpath d='M0 156V40H127.837C148.103 40 164.197 45.1122 176.118 55.3365C188.039 65.5609 194 79.5962 194 97.4423C194 115.103 187.94 129.324 175.82 140.106C163.501 150.702 147.507 156 127.837 156H0Z' fill='%2332C5FF'/%3E%3Cpath d='M54.8659 136V96.0774L73.1454 119.714H77.9906L96.2701 96.0774V136H111.136V60H106.291L75.568 100.488L44.8452 60H40V136H54.8659Z' fill='white'/%3E%3Cpath d='M135.241 136V108.011C135.241 103.251 136.554 99.6253 139.181 97.1324C141.808 94.6394 145.416 93.393 150.004 93.393H154V79.9084C152.594 79.4551 150.966 79.2285 149.116 79.2285C142.826 79.2285 137.794 81.7215 134.02 86.7073V79.9084H120.256V136H135.241Z' fill='white'/%3E%3Cpath d='M284 98V1H244V98H284Z' fill='%233D568A'/%3E%3Cpath d='M284 197V98H244V197H284Z' fill='%2332C5FF'/%3E%3Cpath d='M401.669 119.703L469 1H424.805L381.5 78.7506L338.492 1H294L361.627 120L401.669 119.703Z' fill='%2332C5FF'/%3E%3Cpath d='M402 197V119.261L381.852 78L362 119.56V197H402Z' fill='%233D568A'/%3E%3C/svg%3E"); }
    .small { font-size: 11px; color:#32c5ff; }
    
    /* Status bar */
    .statusbar {
      position: fixed;
      bottom: 0;
      left: 0;
      right: 0;
      background: #ffffff;
      border-top: 1px solid #ddd;
      padding: 8px 16px;
      display: flex;
      align-items: center;
      justify-content: flex-start;
      gap: 16px;
      font-size: 12px;
      z-index: 1000;
      pointer-events: none; /* do not block links/content underneath */
    }
    .status-indicators {
      display: flex;
      gap: 8px;
    }
    .indicator {
      font-size: 13px;
      color: #ccc;
    }
    .indicator.active {
      color: #0a0;
    }
    .indicator.error {
      color: #ff6432;
    }
    .status-stats {
      display: flex;
      gap: 16px;
      color: var(--muted);
      pointer-events: auto;
    }
    .status-messages {
      flex: 0 1 auto;
      color: #a00;
      text-align: left;
      pointer-events: auto;
      margin-right: auto;
    }
    body {
      margin-bottom: 50px;
    }
    
    /* Fuel gauge styling */
    .fuel-gauge {
      text-align: center;
    }
    .fuel-container {
      position: relative;
      display: inline-block;
    }
    .fuel-value {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      font-size: 18px;
      font-weight: bold;
      color: var(--ink);
    }
    #fuel-canvas {
      display: block;
      margin: 0 auto;
    }
    
    /* Top gauges layout */
    .top-gauges {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 20px;
      padding: 0 20px;
      background: #f8f9fa;
      border-radius: 12px;
      width: 100%;
    }
    .gauge-container {
      text-align: center;
      position: relative;
    }
    /* Ensure top gauges are same size on desktop/tablet */
    .gauge-container canvas { width: 200px !important; height: 200px !important; }
    /* Mobile tweaks: place AFTER desktop size so it overrides */
    @media (max-width: 768px) {
      .top-gauges { flex-wrap: wrap; justify-content: center; gap: 12px; }
      .gauge-container { width: 140px !important; height: 140px !important; }
      .gauge-container canvas { width: 140px !important; height: 140px !important; }
      .gauge-container .label { font-size: 14px !important; }
      .gauge-container .fuel-value { font-size: 16px !important; }
    }
    .gauge-container .label {
      font-size: 16px;
      font-weight: 600;
      color: var(--ink);
      margin-bottom: 10px;
    }
    .gauge-container .fuel-value {
      position: absolute;
      top: 56%;
      left: 51%;
      transform: translate(-50%, -50%);
      font-size: 18px;
      font-weight: bold;
      color: var(--ink);
      z-index: 10;
    }
    .metrics-section {
      margin-top: 20px;
    }
    .grid {
      gap: 20px;
      margin-bottom: 20px;
    }
    .panel {
      margin-bottom: 10px;
    }

  </style>
  <link rel="icon" type="image/png" sizes="16x16" href="/favicon.ico" />
  <script>
    function drawGauge(canvas, value, min, max, opts){
      const dpr = window.devicePixelRatio||1; const w = canvas.clientWidth, h = canvas.clientWidth; // square
      canvas.width = w*dpr; canvas.height = h*dpr; const ctx = canvas.getContext('2d'); ctx.scale(dpr,dpr);
      ctx.clearRect(0,0,w,h);
      const cx=w/2, cy=h/2, r=w*0.42, start= Math.PI*0.75, end=Math.PI*2.25, range=end-start;
      // background arc
      ctx.lineCap='round'; ctx.lineWidth = Math.max(10, w*0.06);
      ctx.strokeStyle = '#e9eef5'; ctx.beginPath(); ctx.arc(cx,cy,r,start,end,false); ctx.stroke();
      // value arc
      const pct = Math.max(0, Math.min(1,(value-min)/(max-min)));
      const valEnd = start + range*pct;
      const grad = ctx.createLinearGradient(cx-r,cy, cx+r,cy); grad.addColorStop(0,'#18a0fb'); grad.addColorStop(1,'#32c5ff');
      ctx.strokeStyle = grad; ctx.beginPath(); ctx.arc(cx,cy,r,start,valEnd,false); ctx.stroke();
      // ticks
      ctx.save(); ctx.translate(cx,cy); ctx.rotate(start);
      const steps = opts && opts.ticks || 10; ctx.fillStyle = '#99a8b8';
      for (let i=0;i<=steps;i++){
        const a = i/steps*range; const x = Math.cos(a)*(r+6), y=Math.sin(a)*(r+6);
        ctx.beginPath(); ctx.arc(x,y,1.5,0,Math.PI*2); ctx.fill();
      }
      ctx.restore();
      // label
      ctx.fillStyle='#123'; ctx.textAlign='center'; ctx.textBaseline='middle';
      ctx.font = '700 '+Math.round(w*0.18)+'px system-ui'; ctx.fillText((opts && opts.format?opts.format(value):Math.round(value)), cx, cy);
      ctx.font = '12px system-ui'; ctx.fillStyle='#567'; ctx.fillText(opts&&opts.subtitle||'', cx, cy+Math.round(w*0.2));
    }
    
    function drawFuelGauge(canvas, value) {
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth, h = canvas.clientHeight;
      canvas.width = w * dpr; canvas.height = h * dpr;
      const ctx = canvas.getContext('2d'); ctx.scale(dpr, dpr);
      
      ctx.clearRect(0, 0, w, h);
      const cx = w/2, cy = h/2, r = Math.min(w, h) * 0.38;
      
      // Fuel gauge arc (semi-circle from bottom)
      const start = Math.PI * 0.1; // bottom-left
      const end = Math.PI * 0.9;   // bottom-right
      const range = end - start;
      
      // Background arc
      ctx.lineCap = 'round';
      ctx.lineWidth = Math.max(8, w * 0.04);
      ctx.strokeStyle = '#f0f0f0';
      ctx.beginPath();
      ctx.arc(cx, cy, r, start, end, false);
      ctx.stroke();
      
      // Value arc with color based on fuel level
      const pct = Math.max(0, Math.min(1, value / 100));
      const valEnd = start + range * pct;
      
      // Color gradient: red (low) -> yellow -> green (full)
      let color;
      if (pct < 0.25) color = '#ff4444';      // Red for low fuel
      else if (pct < 0.5) color = '#ffaa00';  // Orange for quarter
      else if (pct < 0.75) color = '#ffdd00'; // Yellow for half
      else color = '#44ff44';                 // Green for full
      
      ctx.strokeStyle = color;
      ctx.lineWidth = Math.max(8, w * 0.04);
      ctx.beginPath();
      ctx.arc(cx, cy, r, start, valEnd, false);
      ctx.stroke();
      
      // Center circle under percentage text
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.arc(cx, cy, r * 0.58, 0, Math.PI * 2);
      ctx.fill();
      
      // E / F labels near ends
      ctx.fillStyle = '#8a99a8';
      ctx.font = Math.round(w * 0.10) + 'px system-ui';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const elx = cx + Math.cos(start) * (r + w*0.06);
      const ely = cy + Math.sin(start) * (r + w*0.06);
      const flx = cx + Math.cos(end) * (r + w*0.06);
      const fly = cy + Math.sin(end) * (r + w*0.06);
      ctx.fillText('E', elx, ely);
      ctx.fillText('F', flx, fly);
    }
    function fmt(n, d=0){ return (n==null?0:n).toFixed(d); }
    function toHMS(sec){ sec = Math.max(0, sec|0); const h=(sec/3600)|0; const m=((sec%3600)/60)|0; const s=sec%60; return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`; }
    document.addEventListener('DOMContentLoaded',()=>{
      const $=id=>document.getElementById(id);
      let en=false; const spdC=$('spd'), rpmC=$('rpm');
      const stateEl=$('obdstate'); const toggleBtn=$('toggle');
      const set=(j)=>{
        drawGauge(spdC, j.speed_kph||0, 0, 240, {subtitle:'km/h'});
        drawGauge(rpmC, j.rpm||0, 0, 8000, {subtitle:'rpm', ticks:8, format:(v)=>String(Math.round(v/100)*100)});
        $('loadv').textContent = fmt(j.load_pct);
        $('fuelv').textContent = fmt(j.fuel_pct);
        $('thrval').textContent = fmt(j.throttle_pct);
        $('coolv').textContent = fmt(j.coolant_c);
        $('mapv').textContent = fmt(j.map_kpa);
        $('iatv').textContent = fmt(j.iat_c);
        $('mafv').textContent = fmt(j.maf_gps,1);
        $('modv').textContent = fmt(j.module_voltage_v,2);
        $('timv').textContent = fmt(j.timing_adv_deg,1);
        // Optional textuals
        const obdStd = document.getElementById('obd-std'); if (obdStd) obdStd.textContent = j.obd_std ?? '';
        const vinEl = document.getElementById('vin'); if (vinEl) vinEl.textContent = j.vin || '';
        const ecuEl = document.getElementById('ecu'); if (ecuEl) ecuEl.textContent = j.ecu_name || '';
        $('barov').textContent = fmt(j.baro_kpa);
        $('ambv').textContent = fmt(j.amb_c);
        $('runv').textContent = toHMS(j.runtime_sec||0);
        $('evapv').textContent = fmt(j.evap_purge_pct);
        
        // Draw fuel gauge
        const fuelCanvas = document.getElementById('fuel-canvas');
        if (fuelCanvas) {
          drawFuelGauge(fuelCanvas, j.fuel_pct || 0);
        }
        
        // bars
        $('bar-thr').style.width = Math.min(100, Math.max(0, j.throttle_pct||0))+'%';
        $('bar-load').style.width = Math.min(100, Math.max(0, j.load_pct||0))+'%';
        $('bar-evap').style.width = Math.min(100, Math.max(0, j.evap_purge_pct||0))+'%';
        const dtc=$('dtc'); dtc.innerHTML=''; (j.dtc||[]).forEach(c=>{ const el=document.createElement('span'); el.className='pill'; el.textContent=c; dtc.appendChild(el); });
        const pidList = $('pidlist'); if (pidList) { pidList.innerHTML=''; (j.pid00_supported||[]).forEach(p=>{ const el=document.createElement('span'); el.className='pill'; el.textContent = '0x'+p; pidList.appendChild(el); }); }
      };
      async function poll(){ try{ const r=await fetch('/obd_poll'); if(r.ok){ const j=await r.json(); en=j.enabled; if(stateEl){ stateEl.textContent = en? 'Enabled':'Disabled'; } if(toggleBtn){ toggleBtn.textContent = en? 'Disable':'Enable'; } set(j); } }catch(e){} finally{ setTimeout(poll,500); } }
      function fmtBps(bps){ const n=Number(bps||0); if(!n) return ''; if(n>=1000000){ const v=n/1000000; const s=(Math.round(v*10)/10); return (Number.isInteger(s)? s.toFixed(0):s.toFixed(1))+' Mbps'; } const v=n/1000; const s=(Math.round(v*10)/10); return (Number.isInteger(s)? s.toFixed(0):s.toFixed(1))+' kbps'; }
      async function pollStats(){ 
        try{ 
          const r=await fetch('/stats'); 
          if(r.ok){ 
            const j=await r.json(); 
            const cs = document.getElementById('canspeed');
            if (cs) cs.textContent = j.can_bps ? `${fmtBps(j.can_bps)}` : '';
            $('rxstats').textContent = `RX: ${j.rx_fps || 0}`; 
            $('txstats').textContent = `TX: ${j.tx_fps || 0}`;
            
            // Update indicators
            const canIndicator = document.getElementById('can-indicator');
            const obdIndicator = document.getElementById('obd-indicator');
            const statusMessages = document.getElementById('status-messages');
            
            if (canIndicator) {
              canIndicator.className = 'indicator' + (j.can_traffic ? ' active' : '');
              canIndicator.title = j.can_traffic ? 'CAN traffic detected' : 'No CAN traffic';
            }
            
            if (obdIndicator) {
              obdIndicator.className = 'indicator' + (j.obd_responses ? ' active' : '');
              obdIndicator.title = j.obd_responses ? 'OBD responses detected' : 'No OBD responses';
            }
            
             if (statusMessages) {
              let msgs = [];
              if (!j.tx_safe) msgs.push('Holding TX (no RX at this speed)');
              if (j.can_errors) msgs.push(j.can_errors);
              if (data.tx_blocked) msgs.push('Read Only mode');
              if (j.tx_suppressed) msgs.push('TX suppressed');
              statusMessages.textContent = msgs.join(' \u2022 ');
            }
          } 
        }catch(e){} 
        finally{ 
          setTimeout(pollStats,500); 
        } 
      }

      // Helpers to enable/disable on demand
      const setEnabled = async (want)=>{
        try{
          await fetch('/obd_toggle',{method:'POST', body: want? 'ENABLE':'DISABLE'});
          en = want;
          if(stateEl) stateEl.textContent = en? 'Enabled':'Disabled';
          if(toggleBtn) toggleBtn.textContent = en? 'Disable':'Enable';
          // inform backend whether to actively TX OBD requests
          try { await fetch('/obd_tx', { method:'POST', body: en? 'ON':'OFF' }); } catch (e) {}
        }catch(e){}
      };
      // Start polling loops
      poll(); pollStats();
      // Auto-enable OBD on page visit (TX will be suppressed automatically if RO)
      setEnabled(true);
      // Manual toggle
      toggleBtn.onclick = async ()=>{ setEnabled(!en); };
    });
  </script>
</head>
<body>
  <header>
    <div style="display:flex;align-items:center;gap:10px">
      <h3 style="margin:0"><img id="logo" alt=""/> OBD-II</h3>
      <span id="obdstate" class="pill">Unknown</span>
      <button id="toggle">Enable</button>
    </div>
    <a class="toplink" href="/">Live CAN</a>
  </header>
  <main class="layout">
    <!-- Speed, RPM, and Fuel at the top -->
    <div class="top-gauges">
      <div class="gauge-container">
        <div class="label">Speed</div>
        <canvas id="spd" width="200" height="200"></canvas>
      </div>
      <div class="gauge-container">
        <div class="label">RPM</div>
        <canvas id="rpm" width="200" height="200"></canvas>
      </div>
      <div class="gauge-container">
        <div class="label">Fuel</div>
        <canvas id="fuel-canvas" width="200" height="200"></canvas>
        <div class="fuel-value"><span id="fuelv">0</span>%</div>
      </div>
    </div>
    
    <!-- All other metrics below -->
    <div class="metrics-section">
      <div class="grid">
        <div class="panel"><div class="label">Throttle</div><div class="bar"><span id="bar-thr" style="width:0%"></span></div><div class="muted"><span id="thrval">0</span>%</div></div>
        <div class="panel"><div class="label">Engine Load</div><div class="bar"><span id="bar-load" style="width:0%"></span></div><div class="muted"><span id="loadv">0</span>%</div></div>
        <div class="panel"><div class="label">EVAP Purge</div><div class="bar"><span id="bar-evap" style="width:0%"></span></div><div class="muted"><span id="evapv">0</span>%</div></div>
      </div>
      <div class="grid">
        <div class="panel"><div class="label">Coolant</div><div class="value"><span id="coolv">0</span><span class="unit">°C</span></div></div>
        <div class="panel"><div class="label">MAP</div><div class="value"><span id="mapv">0</span><span class="unit">kPa</span></div></div>
        <div class="panel"><div class="label">IAT</div><div class="value"><span id="iatv">0</span><span class="unit">°C</span></div></div>
        <div class="panel"><div class="label">MAF</div><div class="value"><span id="mafv">0</span><span class="unit">g/s</span></div></div>
        <div class="panel"><div class="label">Battery Voltage</div><div class="value"><span id="modv">0.00</span><span class="unit">V</span></div></div>
        <div class="panel"><div class="label">Timing Adv.</div><div class="value"><span id="timv">0</span><span class="unit">°</span></div></div>
        <div class="panel"><div class="label">Barometric</div><div class="value"><span id="barov">0</span><span class="unit">kPa</span></div></div>
        <div class="panel"><div class="label">Ambient</div><div class="value"><span id="ambv">0</span><span class="unit">°C</span></div></div>
        <div class="panel"><div class="label">Runtime (MIL)</div><div class="value"><span id="runv">00:00:00</span></div></div>
        <div class="panel"><div class="label">OBD Standard</div><div class="value"><span id="obd-std">-</span></div></div>
        <div class="panel"><div class="label">VIN</div><div class="value"><span id="vin">-</span></div></div>
        <div class="panel"><div class="label">ECU Name</div><div class="value"><span id="ecu">-</span></div></div>
      </div>
      <div class="panel">
        <div class="label">Supported PIDs (01–20)</div>
        <div id="pidlist" class="dtc"></div>
      </div>
      <div class="panel"><div class="label">Fault Codes</div><div id="dtc" class="dtc"></div></div>
    </div>
  </main>
  <div id="statusbar" class="statusbar">
    <div class="status-messages" id="status-messages"></div>
    <div class="status-indicators">
      <span class="indicator" id="can-indicator">● <span class="">CAN</span></span>
      <span class="indicator" id="obd-indicator">● <span class="">OBDII</span></span>
    </div>
    <div class="status-stats">
      <span id="canspeed"></span>
      <span id="rxstats">RX: 0</span>
      <span id="txstats">TX: 0</span>
      <span>frames per second</span>
    </div>
  </div>
</body></html>
)_O";

// ===================== Logging =====================
//#define DEBUG 1
#if DEBUG
  #define LOGF(...)    Serial.printf(__VA_ARGS__)
  #define LOGLN(...)   Serial.println(__VA_ARGS__)
#else
  #define LOGF(...)
  #define LOGLN(...)
#endif

// ===================== Web UI (inline) =====================
// Very small, dependency-free dashboard: HTTP polling binary stream for RX, HTTP POST for TX
static const char INDEX_HTML[] PROGMEM = R"_I(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <link rel="icon" type="image/png" sizes="16x16" href='data:image/x-icon;base64,AAABAAEAEBAAAAEAIABoBAAAFgAAACgAAAAQAAAAIAAAAAEAIAAAAAAAQAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAIdUOc6IVTvpiFU654hVOuaIVTvkiVU74IdUOsqFUDaMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACIVDvli1c9/4pWPf+KVj3/ilY9/4pXPf+LVz3/jVk+/4lWPPaBSS5pAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgk064oVRPf+EUDz/hFA8/4RQPP98Rz3/fUk8/4VRPP+KVzz/jVk+/4ROMXwAAAAA/8s1///LNf//yzX//8s1///KM///yjL//8oy///KMv//yjL//8ky//C4Nf+wfjr/f0o8/4pWPP+LVz3/AAAAAP/FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xjP//8wy/76LOf+DTzz/jFg9/4ZTOLX/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xzP/f0s8/4pWPP+IVTvj/8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8oy/6JwO/+HUzz/ilY88//FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///KMv+fbTv/h1Q8/4pWPPP/xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///FM///xTP//8Uz///GM///xTP/fUg8/4pWPP+JVjri/8Yz///GM///xjP//8Yz///GM///xjP//8Yz///GM///xjP//8Yz///HM///yzL/sX46/4RRPP+MWD3/h1I4sf/GMvT/xjL0/8Yy9P/FMvP4wDP99740//e+NP/3vjT/9740//O6NP/dpzf/m2k7/4JOPP+KVjz/iVY8/wAAAAAAAAAAAAAAAAAAAAAAAAAAd0A74X5IPf98Rzz/fEc8/3xHPP99SDz/gEw8/4dTPP+LVz3/jFg9/4FKL2sAAAAAAAAAAAAAAAAAAAAAAAAAAIhVOuaMWD7/i1c9/4tXPf+LVz3/i1c9/4xYPf+OWT//iFQ75XxDJ0gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACHUji2iFQ6zohTOc2HVDnMh1M5yodUOsaGUjerf0YqVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA//8AAPAPAADwAwAA8AEAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAA8AEAAPADAADwDwAA//8AAA=='/>
  <link rel="prefetch" href="/msg" />
 <title>MrDIY - Live CAN with OBD-II</title>
  <style>
    :root{--brand:#32c5ff;--ink:#123;--muted:#567;--bg:#f7f9fb}
    *{ box-sizing: border-box }
    body { font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; margin: 0; background: var(--bg); color: var(--ink); }
    header{background:#ffffff;color:var(--brand);border-bottom: 2px solid var(--brand);padding:10px 16px;margin-bottom:10px;display:flex;align-items:center;gap:16px}    header h1 { font-size: 16px; margin: 0; }
    h3{margin: 0 0 15px 0;}
    main { padding: 12px; max-width: 1024px; margin: 0 auto; }
    .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
    input, button, select { font-size: 14px; padding:6px 8px; }
    section { background:#fff; border-radius:12px; box-shadow:0 1px 6px rgba(0,0,0,.06); padding:12px }
    table { border-collapse: collapse; width: 100%; font-size: 12px; margin-top:20px; background:#fff; overflow:hidden }
    th, td { border: 1px solid #ddd; padding: 4px 6px; text-align: left; }
    th { background: #f4f4f4; position: sticky; top: 0; z-index: 2; }
    th.sortable { cursor: pointer; }
    tr:nth-child(even) { background: #fafafa; }
    .byte { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; padding: 0 2px; display:inline-block; min-width: 20px; }
    .changed { background: #fffbcc; transition: background 600ms ease-in; }
    .pill { background:#eee; padding:2px 6px; border-radius:10px; font-size:11px; margin-left:40px}
    .ok { color: #0a0; }
    .bad { color: #a00; }
    .grid { display:grid; grid-template-columns: 2fr 1fr; gap: 12px; }
    @media (max-width: 800px) { .grid { grid-template-columns: 1fr; } }
    .small { font-size: 11px; color:#32c5ff; }
    .dsmall { font-size: 11px; }
    #logo{ padding-left:55px;background-size: 37px 16px;height: 17px; background-repeat: no-repeat;background-image: url("data:image/svg+xml,%3Csvg width='469' height='197' viewBox='0 0 469 197' fill='none' xmlns='http://www.w3.org/2000/svg'%3E%3Crect width='469' height='197' fill='%23F2F2F2'/%3E%3Crect width='469' height='197' fill='white'/%3E%3Cpath d='M127.125 196C158.396 196 184.026 186.794 204.016 168.382C224.005 149.97 234 126.212 234 97.1091C234 68.0061 224.104 44.5455 204.312 26.7273C184.521 8.90909 158.792 0 127.125 0H63V196H127.125Z' fill='%233D568A'/%3E%3Cpath d='M0 156V40H127.837C148.103 40 164.197 45.1122 176.118 55.3365C188.039 65.5609 194 79.5962 194 97.4423C194 115.103 187.94 129.324 175.82 140.106C163.501 150.702 147.507 156 127.837 156H0Z' fill='%2332C5FF'/%3E%3Cpath d='M54.8659 136V96.0774L73.1454 119.714H77.9906L96.2701 96.0774V136H111.136V60H106.291L75.568 100.488L44.8452 60H40V136H54.8659Z' fill='white'/%3E%3Cpath d='M135.241 136V108.011C135.241 103.251 136.554 99.6253 139.181 97.1324C141.808 94.6394 145.416 93.393 150.004 93.393H154V79.9084C152.594 79.4551 150.966 79.2285 149.116 79.2285C142.826 79.2285 137.794 81.7215 134.02 86.7073V79.9084H120.256V136H135.241Z' fill='white'/%3E%3Cpath d='M284 98V1H244V98H284Z' fill='%233D568A'/%3E%3Cpath d='M284 197V98H244V197H284Z' fill='%2332C5FF'/%3E%3Cpath d='M401.669 119.703L469 1H424.805L381.5 78.7506L338.492 1H294L361.627 120L401.669 119.703Z' fill='%2332C5FF'/%3E%3Cpath d='M402 197V119.261L381.852 78L362 119.56V197H402Z' fill='%233D568A'/%3E%3C/svg%3E"); }
    .toplink{color:var(--brand);margin-left:auto;text-decoration:none}
      /* Status bar */
    .statusbar {
      position: fixed;
      bottom: 0;
      left: 0;
      right: 0;
      background: #ffffff;
      border-top: 1px solid #ddd;
      padding: 8px 16px;
      display: flex;
      align-items: center;
      justify-content: flex-end; /* align everything to the right */
      gap: 16px;
      font-size: 12px;
      z-index: 1000;
      pointer-events: none; /* do not block links/content underneath */
    }
    .status-indicators {
      display: flex;
      gap: 8px;
    }
    .indicator {
      font-size: 13px;
      color: #ccc;
    }
    .indicator.active {
      color: #0a0;
    }
    .indicator.error {
      color: #ff6432;
    }
    .status-stats {
      display: flex;
      gap: 16px;
      color: var(--muted);
      pointer-events: auto;
    }
    .status-messages {
      flex: 0 0 auto;
      color: #a00;
      text-align: right;
      pointer-events: auto;
    }
    body {
      margin-bottom: 50px;
    }
    #sentCount{
      color:rgb(5, 85, 5);
    }
  </style>
</head>
<body>
      <header>
      <h1><img id="logo" alt=""/>Live CAN</h1>
      <div id="status" class="pill">Connecting ...</div>
      <a class="toplink" href="/obd">OBD‑II</a>
    </header>
  <main>
    <div class="grid">
      <section>
        <h3>Transmit</h3>
        <div class="row">
          <label>ID</label>
          <input id="txId" value="7DF" size="3" />
          <label>EXT</label>
          <select id="txExt">
            <option value="0">0</option>
            <option value="1">1</option>
          </select>
          <label>L</label>
          <select id="txDlc">
            <option value="0">0</option>
            <option value="1">1</option>
            <option value="2">2</option>
            <option value="3">3</option>
            <option value="4">4</option>
            <option value="5">5</option>
            <option value="6">6</option>
            <option value="7">7</option>
            <option value="8" selected>8</option>
          </select>
          <label>Data</label>
          <input id="txB0" size="2" maxlength="2" value="02" placeholder="00" />
          <input id="txB1" size="2" maxlength="2" value="01" placeholder="00" />
          <input id="txB2" size="2" maxlength="2" value="0D" placeholder="00" />
          <input id="txB3" size="2" maxlength="2" value="00" placeholder="00" />
          <input id="txB4" size="2" maxlength="2" value="00" placeholder="00" />
          <input id="txB5" size="2" maxlength="2" value="00" placeholder="00" />
          <input id="txB6" size="2" maxlength="2" value="00" placeholder="00" />
          <input id="txB7" size="2" maxlength="2" value="00" placeholder="00" />
          <label>Every (ms):</label>
          <input id="txEvery" placeholder="0" size="5" />
          <button id="sendBtn">Once</button>
          <button id="toggleRepeat">Repeat</button>
          <span class="muted" style="margin-left:8px;">Sent: <span id="sentCount">0</span></span>
        </div>
      </section>
      <section>
        <h3>Controls</h3>
        <div class="row">
          <label>Update rate:</label>
          <select id="rate">
            <option value="20">20 ms</option>
            <option value="50" selected>50 ms</option>
            <option value="100">100 ms</option>
            <option value="200">200 ms</option>
          </select>
          <button id="applyRate">Apply</button>
        </div>
        <div class="row" style="margin-top:6px;">
          <label>CAN speed:</label>
          <select id="canSpeed">
            <option value="1000000" >1,000,000</option>
            <option value="500000" >500,000</option>
            <option value="250000">250,000</option>
            <option value="125000">125,000</option>
            <option value="100000">100,000</option>
          </select>
          <button id="applySpeed">Apply</button>
        </div>
      </section>
    </div>

    <table id="tbl">
      <thead>
        <tr>
          <th class="sortable" data-col="id">ID</th>
          <th class="sortable" data-col="ext">EXT</th>
          <th class="sortable" data-col="dlc">DLC</th>
          <th class="sortable" data-col="data">DATA</th>
          <th class="sortable" data-col="count">COUNT</th>
          <th class="sortable" data-col="age">AGE (ms)</th>
        </tr>
      </thead>
      <tbody id="tbody"></tbody>
    </table>
  </main>

  <script>
    document.addEventListener('DOMContentLoaded', () => {
      const statusEl = document.getElementById('status');
      const tbody = document.getElementById('tbody');
      const rxstats = document.getElementById('rxstats');
      const txstats = document.getElementById('txstats');
      const rateSel = document.getElementById('rate');
      const btnApplyRate = document.getElementById('applyRate');
      const selSpeed = document.getElementById('canSpeed');
    const btnApplySpeed = document.getElementById('applySpeed');

    function fmtHex(n, width) { return n.toString(16).toUpperCase().padStart(width, '0'); }
    function fmtBps(bps) {
      const n = Number(bps||0);
      if (!n) return '';
      if (n >= 1000000) {
        const v = n / 1000000;
        return (Math.round(v*10)/10).toFixed(v % 1 === 0 ? 0 : 1) + ' Mbps';
      }
      const v = n / 1000;
      return (Math.round(v*10)/10).toFixed(v % 1 === 0 ? 0 : 1) + ' kbps';
    }

    // Frame state map: key -> {id, ext, dlc, data[8], count, ts}
    const frames = new Map();
    const dirtyKeys = new Set();
    let rxCount = 0;
    let txCount = 0;
    let lastTick = performance.now();

    // Sorting state
    let sortKey = 'id';
    let sortAsc = true;
    const thead = document.querySelector('#tbl thead');
    thead.addEventListener('click', (ev) => {
      const th = ev.target.closest('th');
      if (!th || !th.dataset.col) return;
      const col = th.dataset.col;
      if (sortKey === col) {
        sortAsc = !sortAsc;
      } else {
        sortKey = col; sortAsc = true;
      }
      // Force a reorder on next frame
      dirtyKeys.add('__all__');
    });

    // Load saved UI prefs
    let rateMs = parseInt(localStorage.getItem('rateMs')||'50',10);
    rateSel.value = String(rateMs);
    const savedBps = localStorage.getItem('canBps');
    if (savedBps) { selSpeed.value = savedBps; }
    statusEl.textContent = 'Polling';
    statusEl.classList.add('ok');

    let pollStopped = false;
    window.__stopLoops = () => { pollStopped = true; };
    async function poll() {
      if (pollStopped) return;
      try {
        const res = await fetch('/poll');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const buf = await res.arrayBuffer();
        const dv = new DataView(buf);
        let off = 0;
        const header = dv.getUint8(off); off += 1; // 0xB1 stream batch
        if (header === 0xB1) {
          const count = dv.getUint16(off, true); off += 2;
          const now = performance.now();
          for (let i = 0; i < count; i++) {
            const key = dv.getUint32(off, true); off += 4;
            const flags = dv.getUint8(off); off += 1;
            const ext = (flags & 0x80) ? 1 : 0;
            const dlc = (flags & 0x0F);
            const id = key & 0x1FFFFFFF;
            const data = new Uint8Array(buf, off, 8);
            off += 8;
            const prev = frames.get(key);
            if (!prev) {
              frames.set(key, { id, ext, dlc, data: new Uint8Array(data), count: 1, ts: now });
            } else {
              const newData = new Uint8Array(data);
              for (let b = 0; b < dlc; b++) if (prev.data[b] !== newData[b]) {}
              prev.data.set(newData);
              prev.dlc = dlc; prev.count++; prev.ts = now;
            }
            dirtyKeys.add(key);
            rxCount++;
          }
        }
      } catch (e) {
        statusEl.textContent = 'Error'; statusEl.classList.add('bad');
      } finally {
        if (!pollStopped) setTimeout(poll, rateMs);
      }
    }
    poll();

    function render() {
      const now = performance.now();
      const age = (t) => Math.round(now - t);
              // Update stats once per 500ms
        if (!pollStopped && now - lastTick >= 500) {
          // Fetch stats from server (server calculates FPS)
          fetch('/stats').then(r => r.json()).then(data => {
            const cans = document.getElementById('canspeed');
            if (cans && data.can_bps) cans.textContent = `${fmtBps(data.can_bps)}`;
            rxstats.textContent = `RX: ${data.rx_fps || 0}`;
            txstats.textContent = `TX: ${data.tx_fps || 0}`;
            
            // Update indicators
            const canIndicator = document.getElementById('can-indicator');
            const obdIndicator = document.getElementById('obd-indicator');
            const statusMessages = document.getElementById('status-messages');
            
            if (canIndicator) {
              canIndicator.className = 'indicator' + (data.can_traffic ? ' active' : '');
              canIndicator.title = data.can_traffic ? 'CAN traffic detected' : 'No CAN traffic';
            }
            
            if (obdIndicator) {
              obdIndicator.className = 'indicator' + (data.obd_responses ? ' active' : '');
              obdIndicator.title = data.obd_responses ? 'OBD responses detected' : 'No OBD responses';
            }
            
             if (statusMessages) {
               let msgs = [];
               if (data && data.tx_safe === false) msgs.push('Holding TX (no RX at this speed)');
               if (data.can_errors) msgs.push(data.can_errors);
               if (data.tx_blocked) msgs.push('Read Only mode');
               if (data.tx_suppressed) msgs.push('TX suppressed');
               statusMessages.textContent = msgs.join(' \u2022 ');
             }
          }).catch(() => {
            rxstats.textContent = `RX: 0`;
            txstats.textContent = `TX: 0`;
          });
          lastTick = now;
        }

      // Apply row updates for dirty keys only
      dirtyKeys.forEach((key) => {
        const f = frames.get(key);
        if (!f) return;
        let row = document.getElementById('r-' + key);
        if (!row) {
          row = document.createElement('tr');
          row.id = 'r-' + key;
          row.innerHTML = '<td class="id"></td><td class="ext"></td><td class="dlc"></td><td class="data"></td><td class="count"></td><td class="age"></td>';
          tbody.appendChild(row);
        }
        const idText = f.ext ? ('0x' + fmtHex(f.id, 8)) : ('0x' + fmtHex(f.id, 3));
        row.querySelector('.id').innerHTML = `<a href="/msg?id=${f.id}&ext=${f.ext}" onclick="window.__stopLoops&&window.__stopLoops()" style="text-decoration:none">${idText}</a>`;
        row.querySelector('.ext').textContent = f.ext;
        row.querySelector('.dlc').textContent = f.dlc;
        const dataCell = row.querySelector('.data');
        // build bytes with highlight
        let html = '';
        for (let i = 0; i < f.dlc; i++) {
          html += `<span class="byte" id="${'b-' + key + '-' + i}">${fmtHex(f.data[i], 2)}</span>`;
          if (i < f.dlc - 1) html += ' ';
        }
        dataCell.innerHTML = html;
        row.querySelector('.count').textContent = f.count;
        row.querySelector('.age').textContent = age(f.ts).toString();
      });
      // Flash changed bytes
      dirtyKeys.forEach((key) => {
        const f = frames.get(key);
        if (!f) return;
        for (let i = 0; i < f.dlc; i++) {
          const el = document.getElementById('b-' + key + '-' + i);
          if (el) { el.classList.add('changed'); setTimeout(() => el.classList.remove('changed'), 150); }
        }
      });
      // Reorder rows according to sort
      if (dirtyKeys.size > 0) {
        const items = Array.from(frames.entries());
        const now2 = performance.now();
        items.sort((a,b) => {
          const fa = a[1], fb = b[1];
          let va = 0, vb = 0, cmp = 0;
          switch (sortKey) {
            case 'id': cmp = (fa.id - fb.id); break;
            case 'ext': cmp = (fa.ext - fb.ext); break;
            case 'dlc': cmp = (fa.dlc - fb.dlc); break;
            case 'count': cmp = (fa.count - fb.count); break;
            case 'age': {
              const aa = now2 - fa.ts, ab = now2 - fb.ts;
              cmp = aa - ab; break;
            }
            case 'data': {
              for (let i = 0; i < Math.max(fa.dlc, fb.dlc); i++) {
                const da = i < fa.dlc ? fa.data[i] : -1;
                const db = i < fb.dlc ? fb.data[i] : -1;
                if (da !== db) { cmp = da - db; break; }
              }
              break;
            }
            default: cmp = 0;
          }
          return sortAsc ? cmp : -cmp;
        });
        for (const [key] of items) {
          const row = document.getElementById('r-' + key);
          if (row) tbody.appendChild(row);
        }
      }
      dirtyKeys.clear();
      requestAnimationFrame(render);
    }
    requestAnimationFrame(render);

    // TX handling
    const elId = document.getElementById('txId');
    const elExt = document.getElementById('txExt');
    const elDlc = document.getElementById('txDlc');
    const elBytes = Array.from({length:8}, (_,i)=>document.getElementById('txB'+i));
    const elEvery = document.getElementById('txEvery');
    const btnToggle = document.getElementById('toggleRepeat');
    const elSentCount = document.getElementById('sentCount');
    let repeatTimer = null;
    let sentCount = 0;
    document.getElementById('sendBtn').onclick = async () => {
      const ext = parseInt(elExt.value) ? 1 : 0;
      const dlc = Math.max(0, Math.min(8, parseInt(elDlc.value) || 0));
      const id = parseInt(elId.value, 16);
      const bytes = elBytes.map(inp => {
        const v = (inp && inp.value || "").trim();
        const n = parseInt(v || "0", 16);
        return isFinite(n) ? (n & 0xFF) : 0;
      });
      const cmd = `TX ${id} ${ext} ${dlc} ${bytes.map(b => b.toString(16).padStart(2,'0')).join('')}`;
      try { await fetch('/tx', { method:'POST', body: cmd }); sentCount++; if (elSentCount) elSentCount.textContent = String(sentCount); } catch {}
    };

    function buildTxCommand() {
      const ext = parseInt(elExt.value) ? 1 : 0;
      const dlc = Math.max(0, Math.min(8, parseInt(elDlc.value) || 0));
      const id = parseInt(elId.value, 16);
      const bytes = elBytes.map(inp => {
        const v = (inp && inp.value || "").trim();
        const n = parseInt(v || "0", 16);
        return isFinite(n) ? (n & 0xFF) : 0;
      });
      return `TX ${id} ${ext} ${dlc} ${bytes.map(b => b.toString(16).padStart(2,'0')).join('')}`;
    }

    btnToggle.onclick = async () => {
      if (repeatTimer) {
        clearInterval(repeatTimer);
        repeatTimer = null;
        btnToggle.textContent = 'Start';
        return;
      }
      const every = Math.max(0, parseInt(elEvery.value, 10) || 0);
      if (every <= 0) return;
      // do not reset counter on start; continue counting
      btnToggle.textContent = 'Stop';
      repeatTimer = setInterval(async () => {
        try { await fetch('/tx', { method:'POST', body: buildTxCommand() }); sentCount++; if (elSentCount) elSentCount.textContent = String(sentCount); } catch {}
      }, every);
    };

    btnApplyRate.onclick = () => { rateMs = parseInt(rateSel.value, 10) || 50; localStorage.setItem('rateMs', String(rateMs)); };
    btnApplySpeed.onclick = async () => {
      const spd = parseInt(selSpeed.value, 10) || 500000;
      try { await fetch('/speed', { method:'POST', body: String(spd) }); localStorage.setItem('canBps', String(spd)); } catch {}
    };
    });
  </script>
  <div id="statusbar" class="statusbar">
    <div class="status-messages" id="status-messages"></div>
    <div class="status-indicators">
      <span class="indicator" id="can-indicator">● <span class="">CAN</span></span>
      <span class="indicator" id="obd-indicator">● <span class="">OBDII</span></span>
    </div>
    <div class="status-stats">
      <span id="canspeed"></span>
      <span id="rxstats">RX: 0</span>
      <span id="txstats">TX: 0</span>
      <span>frames per second</span>
    </div>
  </div>
</body>
</html>
)_I";

// ===================== Message Monitor Page =====================
static const char MSG_HTML[] PROGMEM = R"_M(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Message Monitor</title>
  <style>
    :root{--brand:#32c5ff;--ink:#123;--muted:#567;--bg:#f7f9fb}
    *{box-sizing:border-box}
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:var(--bg);color:var(--ink)}
    header{background:#ffffff;color:var(--brand);border-bottom:2px solid var(--brand);padding:10px 16px;margin-bottom:10px;display:flex;align-items:center;gap:16px}
    header h1{font-size:16px;margin:0}
    .toplink{color:var(--brand);margin-left:auto;text-decoration:none}
    main{padding:14px;max-width:1024px;margin:0 auto}
    .card{background:#fff;border-radius:12px;box-shadow:0 1px 6px rgba(0,0,0,.06);padding:12px}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    @media (max-width: 800px){.grid{grid-template-columns:1fr}}
    .mono{font-family:ui-monospace, SFMono-Regular, Menlo, Consolas, monospace}
    .bytes{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px}
    .byte{display:inline-block;min-width:24px;padding:2px 4px;border-radius:4px;background:#f4f7fb}
    .byte.changed{background:#fffbcc;}
    .kv{color:var(--muted);font-size:12px}
    .big{font-size:22px;font-weight:700}
    .bitgrid{display:grid;grid-template-columns: repeat(8, 1fr); gap:4px}
    .bit{background:#eef3f9;border-radius:4px;padding:6px 4px;text-align:center;font-family:ui-monospace, SFMono-Regular, Menlo, Consolas, monospace}
    .bit.changed{background:#fffbcc}
    .field{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-top:10px}
    /* status bar shared */
    .statusbar{position:fixed;bottom:0;left:0;right:0;background:#fff;border-top:1px solid #ddd;padding:8px 16px;display:flex;align-items:center;justify-content:flex-start;gap:16px;font-size:12px;z-index:1000;pointer-events:none}
    .status-indicators{display:flex;gap:8px}
    .indicator{font-size:13px;color:#ccc}
    .indicator.active{color:#0a0}
    .status-stats{display:flex;gap:16px;color:var(--muted);pointer-events:auto}
    .status-messages{flex:0 1 auto;color:#a00;text-align:left;pointer-events:auto;margin-right:auto}
    body{margin-bottom:50px}
    #logo{ padding-left:55px;background-size: 37px 16px;height: 17px; background-repeat: no-repeat;background-image: url("data:image/svg+xml,%3Csvg width='469' height='197' viewBox='0 0 469 197' fill='none' xmlns='http://www.w3.org/2000/svg'%3E%3Crect width='469' height='197' fill='%23F2F2F2'/%3E%3Crect width='469' height='197' fill='white'/%3E%3Cpath d='M127.125 196C158.396 196 184.026 186.794 204.016 168.382C224.005 149.97 234 126.212 234 97.1091C234 68.0061 224.104 44.5455 204.312 26.7273C184.521 8.90909 158.792 0 127.125 0H63V196H127.125Z' fill='%233D568A'/%3E%3Cpath d='M0 156V40H127.837C148.103 40 164.197 45.1122 176.118 55.3365C188.039 65.5609 194 79.5962 194 97.4423C194 115.103 187.94 129.324 175.82 140.106C163.501 150.702 147.507 156 127.837 156H0Z' fill='%2332C5FF'/%3E%3Cpath d='M54.8659 136V96.0774L73.1454 119.714H77.9906L96.2701 96.0774V136H111.136V60H106.291L75.568 100.488L44.8452 60H40V136H54.8659Z' fill='white'/%3E%3Cpath d='M135.241 136V108.011C135.241 103.251 136.554 99.6253 139.181 97.1324C141.808 94.6394 145.416 93.393 150.004 93.393H154V79.9084C152.594 79.4551 150.966 79.2285 149.116 79.2285C142.826 79.2285 137.794 81.7215 134.02 86.7073V79.9084H120.256V136H135.241Z' fill='white'/%3E%3Cpath d='M284 98V1H244V98H284Z' fill='%233D568A'/%3E%3Cpath d='M284 197V98H244V197H284Z' fill='%2332C5FF'/%3E%3Cpath d='M401.669 119.703L469 1H424.805L381.5 78.7506L338.492 1H294L361.627 120L401.669 119.703Z' fill='%2332C5FF'/%3E%3Cpath d='M402 197V119.261L381.852 78L362 119.56V197H402Z' fill='%233D568A'/%3E%3C/svg%3E"); }

  </style>
  <link rel="icon" type="image/png" sizes="16x16" href="/favicon.ico" />
  <script>
    function fmtHex(n, w){return Number(n).toString(16).toUpperCase().padStart(w,'0')}
    function fmtBps(b){const n=Number(b||0);if(!n) return '';if(n>=1e6){const v=n/1e6;return ((Math.round(v*10)/10).toFixed(v%1===0?0:1)+' Mbps')}const v=n/1e3;return ((Math.round(v*10)/10).toFixed(v%1===0?0:1)+' kbps')}
    function bytesToBits(arr){ const bits=[]; for(let i=0;i<8;i++){ const v=arr[i]||0; for(let b=7;b>=0;b--){ bits.push((v>>b)&1); } } return bits; }
    function extractBits(bits, start, end, endian){
      const s=Math.max(0, Math.min(63, start|0)), e=Math.max(0, Math.min(63, end|0));
      if (e < s) return {hex:'0', dec:0};
      const len = e - s + 1;
      let val = 0;
      if (endian === 'le') {
        for (let i=0;i<len;i++) { const bitIndex = s + i; const byte = (bitIndex>>3); const off = bitIndex & 7; val |= (((bits[byte*8 + (7-off)]||0)&1) << i); }
      } else {
        for (let i=0;i<len;i++) { const bitIndex = s + i; val = (val<<1) | (bits[bitIndex]||0); }
      }
      const hex = val.toString(16).toUpperCase();
      return {hex, dec:val>>>0};
    }
    document.addEventListener('DOMContentLoaded',()=>{
      const params=new URLSearchParams(location.search); const id=Number(params.get('id')||0); const ext=Number(params.get('ext')||0);
      const idEl=document.getElementById('id'); const extEl=document.getElementById('ext'); const dlcEl=document.getElementById('dlc');
      const countTotal=document.getElementById('countTotal'); const countSeen=document.getElementById('countSeen'); const ageEl=document.getElementById('age'); const bytesEl=document.getElementById('bytes'); const bitGrid=document.getElementById('bitgrid');
      const startBit=document.getElementById('startBit'); const endBit=document.getElementById('endBit'); const endianSel=document.getElementById('endian'); const selHex=document.getElementById('selHex'); const selDec=document.getElementById('selDec');
      const rxstats=document.getElementById('rxstats'); const txstats=document.getElementById('txstats'); const cans=document.getElementById('canspeed');
      const canIndicator=document.getElementById('can-indicator'); const obdIndicator=document.getElementById('obd-indicator'); const statusMessages=document.getElementById('status-messages');
      let lastData=new Array(8).fill(0);
      let lastBits=new Array(64).fill(0);
      let lastCount=0;
      let baseAgeMs=0;
      let baseTick=performance.now();

      idEl.textContent = '0x'+fmtHex(id, ext?8:3);
      extEl.textContent = String(ext);

      async function poll(){
        try{
          const r=await fetch(`/msg_poll?id=${id}&ext=${ext}`);
          if(r.ok){ const j=await r.json(); if (j && j.id!==undefined){
            dlcEl.textContent = String(j.dlc||0);
            const total = Number(j.count||0);
            countTotal.textContent = String(total);
            const d = Array.isArray(j.data)? j.data.slice(0,8):[];
            const total = Number(j.count||0);
            let anyChange = (total !== Number(lastCount||0));
            bytesEl.innerHTML=''; bitGrid.innerHTML='';
            for(let i=0;i<8;i++){
              const v = d[i]||0; const el=document.createElement('span'); el.className='byte mono'; el.textContent=fmtHex(v,2);
              if (v !== lastData[i]) { el.classList.add('changed'); anyChange = true; }
              bytesEl.appendChild(el);
              lastData[i]=v;
            }
            const bits = bytesToBits(d);
            for (let i=0;i<64;i++){
              const be = document.createElement('div'); be.className='bit mono';
              const bv = bits[i]||0; be.textContent = String(bv);
              if (bv !== (lastBits[i]||0)) { be.classList.add('changed'); anyChange = true; }
              bitGrid.appendChild(be);
            }
            lastBits = bits.slice();
            if (anyChange) {
              baseAgeMs = 0;
              baseTick = performance.now();
              // Update local seen counter by incrementing difference
              const prev = Number(lastCount||0);
              const inc = Math.max(0, total - prev);
              countSeen.textContent = String((Number(countSeen.textContent)||0) + inc);
              lastCount = total;
            } else {
              if (typeof j.age_ms === 'number') baseAgeMs = j.age_ms;
              baseTick = performance.now();
            }
            const res = extractBits(bits, parseInt(startBit.value||'0',10), parseInt(endBit.value||'0',10), endianSel.value);
            selHex.textContent = res.hex; selDec.textContent = String(res.dec);
          } }
        }catch(e){}
        setTimeout(poll, 250);
      }
      poll();

      // Smooth age updater
      function tickAge(){
        const now = performance.now();
        ageEl.textContent = String(Math.max(0, Math.round(baseAgeMs + (now - baseTick))));
        requestAnimationFrame(tickAge);
      }
      requestAnimationFrame(tickAge);

      function pollStats(){
        fetch('/stats').then(r=>r.json()).then(j=>{
          if (cans && j.can_bps) cans.textContent = fmtBps(j.can_bps);
          rxstats.textContent = `RX: ${j.rx_fps||0}`; txstats.textContent = `TX: ${j.tx_fps||0}`;
          if (canIndicator) { canIndicator.className = 'indicator' + (j.can_traffic? ' active':''); }
          if (obdIndicator) { obdIndicator.className = 'indicator' + (j.obd_responses? ' active':''); }
          if (statusMessages){ let msgs=[]; if (j.tx_safe===false) msgs.push('Holding TX (no RX at this speed)'); if (j.can_errors) msgs.push(j.can_errors); if (j.tx_blocked) msgs.push('Read Only mode'); if (j.tx_suppressed) msgs.push('TX suppressed'); statusMessages.textContent=msgs.join(' \u2022 '); }
        }).finally(()=> setTimeout(pollStats, 500));
      }
      pollStats();
    });
  </script>
</head>
<body>
  <header>
    <h1><img id="logo" alt=""/>Message</h1>
    <a class="toplink" href="/">Live CAN</a>
  </header>
  <main>
    <div class="grid">
      <div class="card">
        <div class="kv">CAN ID</div>
        <div class="big mono" id="id">-</div>
      </div>
      <div class="card">
        <div class="kv">EXT</div>
        <div class="big mono" id="ext">-</div>
      </div>
      <div class="card">
        <div class="kv">DLC</div>
        <div class="big mono" id="dlc">-</div>
      </div>
      <div class="card">
        <div class="kv">Count</div>
        <div class="big mono"><span id="countTotal">0</span> <span class="kv" style="margin-left:8px">(seen: <span id="countSeen">0</span>)</span></div>
      </div>
      <div class="card">
        <div class="kv">Age (ms)</div>
        <div class="big mono" id="age">0</div>
      </div>
      <div class="card" style="grid-column: 1 / -1;">
        <div class="kv">Data (hex)</div>
        <div id="bytes" class="bytes"></div>
        <div class="kv" style="margin-top:6px">Bits (bit 0..63)</div>
        <div id="bitgrid" class="bitgrid"></div>
      </div>
      <div class="card" style="grid-column: 1 / -1;">
        <div class="kv">Extract</div>
        <div class="field">
          <label>Start bit</label><input id="startBit" size="3" value="0" />
          <label>End bit</label><input id="endBit" size="3" value="7" />
          <label>Endian</label>
          <select id="endian"><option value="be" selected>Big</option><option value="le">Little</option></select>
          <span>Hex:</span><span class="mono" id="selHex">0</span>
          <span>Dec:</span><span class="mono" id="selDec">0</span>
        </div>
      </div>
    </div>
  </main>
  <div id="statusbar" class="statusbar">
    <div class="status-messages" id="status-messages"></div>
    <div class="status-indicators">
      <span class="indicator" id="can-indicator">● <span class="">CAN</span></span>
      <span class="indicator" id="obd-indicator">● <span class="">OBDII</span></span>
    </div>
    <div class="status-stats">
      <span id="canspeed"></span>
      <span id="rxstats">RX: 0</span>
      <span id="txstats">TX: 0</span>
      <span>frames per second</span>
    </div>
  </div>
</body>
</html>
)_M";

// ===================== Aggregator =====================
struct FrameEntry {
  uint32_t key;          // id | (ext<<31)
  uint32_t id;
  uint8_t  ext;
  uint8_t  dlc;
  uint8_t  data[8];
  uint32_t count;
  uint32_t last_ms;
  bool     occupied;
  bool     dirty;
};

static FrameEntry* g_table = nullptr; // will be allocated at runtime (PSRAM if available)
static SemaphoreHandle_t g_tableMutex;

static inline uint32_t now_ms() { return (uint32_t) millis(); }

static inline uint32_t make_key(uint32_t id, uint8_t ext) {
  return (id & 0x1FFFFFFFUL) | (ext ? 0x80000000UL : 0);
}

static inline size_t hash_key(uint32_t key) {
  // Knuth multiplicative hashing
  return (size_t)((key * 2654435761UL) % AGG_TABLE_SIZE);
}

// Lookup a frame entry by key (id | ext<<31). Returns pointer or nullptr.
static FrameEntry* table_find_by_key(uint32_t key) {
  size_t idx = hash_key(key);
  for (size_t probe = 0; probe < AGG_TABLE_SIZE; ++probe) {
    FrameEntry& e = g_table[idx];
    if (!e.occupied) return nullptr; // empty slot stops search
    if (e.key == key) return &e;
    idx = (idx + 1) % AGG_TABLE_SIZE;
  }
  return nullptr;
}

// Determine if it is safe to transmit (recent valid RX seen at current bitrate)
static inline bool can_tx_safely() {
  if (!g_can_running) return false;
  uint32_t now = millis();
  // Require some RX within the last 5000 ms to consider bitrate healthy
  return (now - g_last_can_traffic_ms) <= 5000;
}

static FrameEntry* table_upsert(uint32_t id, uint8_t ext, uint8_t dlc, const uint8_t data[8], bool* out_changed) {
  const uint32_t key = make_key(id, ext);
  size_t idx = hash_key(key);
  for (size_t probe = 0; probe < AGG_TABLE_SIZE; ++probe) {
    FrameEntry& e = g_table[idx];
    if (!e.occupied) {
      e.occupied = true; e.key = key; e.id = id; e.ext = ext; e.dlc = dlc; memcpy(e.data, data, 8);
      e.count = 1; e.last_ms = now_ms(); e.dirty = true; if (out_changed) *out_changed = true; return &e;
    }
    if (e.key == key) {
      bool changed = (e.dlc != dlc) || (memcmp(e.data, data, 8) != 0);
      e.dlc = dlc; memcpy(e.data, data, 8); e.count++; e.last_ms = now_ms(); if (changed) e.dirty = true; if (out_changed) *out_changed = changed; return &e;
    }
    idx = (idx + 1) % AGG_TABLE_SIZE;
  }
  if (out_changed) *out_changed = false;
  return nullptr;
}

// Collect dirty entries into a temporary list for flushing
static size_t table_collect_dirty(FrameEntry** out_list, size_t max_items) {
  size_t n = 0;
  for (size_t i = 0; i < AGG_TABLE_SIZE && n < max_items; ++i) {
    if (g_table[i].occupied && g_table[i].dirty) {
      out_list[n++] = &g_table[i];
    }
  }
  return n;
}

static void table_clear_dirty(FrameEntry** list, size_t n) {
  for (size_t i = 0; i < n; ++i) list[i]->dirty = false;
}

// ===================== HTTP server =====================
static WebServer g_server(80);
// No WebSocket: we will use binary polling endpoint for maximum compatibility

static void root_get_handler() {
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send_P(200, "text/html", INDEX_HTML);
}

static void poll_get_handler() {
  // Collect dirty entries and send as binary batch
  const size_t MAX_BATCH = 512; // reduce stack/malloc size
  FrameEntry* list[MAX_BATCH];
  size_t n = 0;
  if (xSemaphoreTake(g_tableMutex, portMAX_DELAY) == pdTRUE) {
    n = table_collect_dirty(list, MAX_BATCH);
    table_clear_dirty(list, n);
    xSemaphoreGive(g_tableMutex);
  }
  const size_t ENTRY_SZ = 4 + 1 + 8;
  const size_t HEADER_SZ = 1 + 2;
  size_t total = HEADER_SZ + ENTRY_SZ * n;
  uint8_t* payload = (uint8_t*) malloc(total);
  if (!payload) { g_server.send(500, "text/plain", "oom\n"); return; }
  size_t off = 0;
  payload[off++] = 0xB1;
  payload[off++] = (uint8_t)(n & 0xFF);
  payload[off++] = (uint8_t)((n >> 8) & 0xFF);
  for (size_t i = 0; i < n; ++i) {
    FrameEntry* e = list[i];
    payload[off++] = (uint8_t)(e->key & 0xFF);
    payload[off++] = (uint8_t)((e->key >> 8) & 0xFF);
    payload[off++] = (uint8_t)((e->key >> 16) & 0xFF);
    payload[off++] = (uint8_t)((e->key >> 24) & 0xFF);
    uint8_t flags = (e->ext ? 0x80 : 0) | (e->dlc & 0x0F);
    payload[off++] = flags;
    memcpy(&payload[off], e->data, 8); off += 8;
  }
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.setContentLength(total);
  g_server.send(200, "application/octet-stream", "");
  WiFiClient client = g_server.client();
  if (client) client.write(payload, total);
  free(payload);
}

static void tx_post_handler() {
  String body = g_server.arg("plain");
  if (!body.length()) { g_server.send(408, "text/plain", "no body\n"); return; }
  std::unique_ptr<char[]> buf(new char[body.length() + 1]);
  memcpy(buf.get(), body.c_str(), body.length() + 1);
  char* cmd = buf.get();
  if (strncmp(cmd, "TX ", 3) == 0) {
    if (g_tx_suppressed || g_tx_write_blocked) { g_server.send(423, "text/plain", "TX disabled (read-only or errors)\n"); return; }
    char* p = cmd + 3;
    uint32_t id = (uint32_t) strtoul(p, &p, 10);
    while (*p == ' ') p++;
    uint32_t ext = (uint32_t) strtoul(p, &p, 10);
    while (*p == ' ') p++;
    uint32_t dlc = (uint32_t) strtoul(p, &p, 10);
    while (*p == ' ') p++;
    uint8_t data[8] = {0};
    for (uint32_t i = 0; i < 8 && p && *p; i++) {
      char hex[3] = { '0', '0', 0 };
      if (*p) { hex[0] = *p++; }
      if (*p) { hex[1] = *p++; }
      data[i] = (uint8_t) strtoul(hex, nullptr, 16);
    }
    twai_message_t m = {};
    m.identifier = id & 0x1FFFFFFF;
    m.extd = ext ? 1 : 0;
    m.rtr = 0;
    m.data_length_code = (uint8_t) (dlc & 0x0F);
    memcpy(m.data, data, 8);
    esp_err_t txr = twai_transmit(&m, pdMS_TO_TICKS(5));
    if (txr == ESP_OK) { g_tx_count++; g_tx_fail_streak = 0; }
    else { if (g_tx_fail_streak < 255) g_tx_fail_streak++; if (g_tx_fail_streak >= 3) { g_tx_write_blocked = true; g_tx_suppressed = true; } }
  }
  g_server.send(200, "text/plain", "OK\n");
}

// ===================== CAN RX → Aggregator =====================
static void can_rx_task(void* arg) {
  twai_message_t m;
  for (;;) {
    if (g_can_running && twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK) {
      bool changed = false;
      if (xSemaphoreTake(g_tableMutex, portMAX_DELAY) == pdTRUE) {
        table_upsert(m.identifier, m.extd ? 1 : 0, m.data_length_code, m.data, &changed);
        xSemaphoreGive(g_tableMutex);
      }
      g_last_can_rx_ms = millis();
      g_last_can_traffic_ms = millis();
      g_can_traffic_detected = true;
      g_rx_count++;
      // brief activity pulse
      digitalWrite(SHIELD_LED_BLUE_PIN, HIGH);

      // Lightweight OBD-II parse for common PIDs on 0x7E8 responses (ISO-TP single frame only)
      if (xSemaphoreTake(g_obdMutex, 1) == pdTRUE) {
        if (g_obd.enabled && (m.identifier == 0x7E8) && m.data_length_code >= 3) {
          // Mode 01 response: 0x41
          if ((m.data[1] == 0x41)) {
            uint8_t pid = m.data[2];
            g_last_obd_response_ms = millis();
            g_obd_responses_detected = true;
            switch (pid) {
              case 0x04: // Engine load
                if (m.data_length_code >= 4) g_obd.engineLoadPct = (m.data[3] * 100.0f) / 255.0f;
                break;
              case 0x0C: // RPM
                if (m.data_length_code >= 5) {
                  uint16_t v = ((uint16_t)m.data[3] << 8) | m.data[4];
                  g_obd.rpm = v / 4.0f;
                }
                break;
              case 0x0D: // Speed
                if (m.data_length_code >= 4) g_obd.speedKph = m.data[3];
                break;
              case 0x0B: // MAP
                if (m.data_length_code >= 4) g_obd.intakeMapKpa = m.data[3];
                break;
              case 0x0E: // Timing advance (A/2 - 64)
                if (m.data_length_code >= 4) g_obd.timingAdvanceDeg = (m.data[3] / 2.0f) - 64.0f;
                break;
              case 0x0F: // IAT
                if (m.data_length_code >= 4) g_obd.intakeAirTempC = (int)m.data[3] - 40;
                break;
              case 0x10: // MAF (A*256+B)/100 g/s
                if (m.data_length_code >= 6) { uint16_t vv = ((uint16_t)m.data[3] << 8) | m.data[4]; g_obd.mafGps = vv / 100.0f; }
                break;
              case 0x05: // Coolant temp
                if (m.data_length_code >= 4) g_obd.coolantC = (int16_t)m.data[3] - 40;
                break;
              case 0x11: // Throttle position
                if (m.data_length_code >= 4) g_obd.throttlePct = (m.data[3] * 100.0f) / 255.0f;
                break;
              case 0x2F: // Fuel level
                if (m.data_length_code >= 4) g_obd.fuelPct = (m.data[3] * 100.0f) / 255.0f;
                break;
              case 0x00: // Supported PIDs 01-20
                if (m.data_length_code >= 8) {
                  uint32_t mask = ((uint32_t)m.data[3] << 24) | ((uint32_t)m.data[4] << 16) | ((uint32_t)m.data[5] << 8) | (uint32_t)m.data[6];
                  g_obd.pid00Mask = mask; g_obd.pid00Valid = true;
                }
                break;
              case 0x1C: // OBD standards this vehicle conforms to
                if (m.data_length_code >= 4) g_obd.obdStdCode = m.data[3];
                break;
              case 0x33: // Baro
                if (m.data_length_code >= 4) g_obd.baroKpa = m.data[3];
                break;
              case 0x42: // Control module voltage
                if (m.data_length_code >= 5) {
                  uint16_t raw = ((uint16_t)m.data[3] << 8) | m.data[4];
                  g_obd.moduleVoltageV = raw / 1000.0f; // per standard PID 42
                }
                break;
              case 0x1F: // Runtime since engine start (seconds)
                if (m.data_length_code >= 6) { uint16_t vv = ((uint16_t)m.data[3] << 8) | m.data[4]; g_obd.runtimeSec = vv; }
                break;
              case 0x46: // Ambient
                if (m.data_length_code >= 4) g_obd.ambientAirTempC = (int)m.data[3] - 40;
                break;
              case 0x4D: // Time run MIL (A*256+B) seconds
                if (m.data_length_code >= 6) { uint16_t vv = ((uint16_t)m.data[3] << 8) | m.data[4]; g_obd.runtimeSec = vv; }
                break;
              case 0x52: // EVAP purge commanded
                if (m.data_length_code >= 4) g_obd.evapPurgePct = (m.data[3] * 100.0f) / 255.0f;
                break;
              default: break;
            }
          } else if (m.data[1] == 0x49) { // Mode 09 response
            uint8_t pid = m.data[2];
            // Basic single-frame parsing for VIN (0x02) and ECU name (0x0A)
            if (pid == 0x02) {
              // m.data[0]=len, [1]=0x49, [2]=0x02, [3]=F0 record#, [4..] = ASCII chunk
              uint8_t dataCount = m.data_length_code - 3;
              const uint8_t* src = &m.data[3];
              // Skip the record index byte if present (common ECUs put it at [3])
              if (dataCount > 0 && (src[0] & 0xF0)) { src++; dataCount--; }
              for (uint8_t i = 0; i < dataCount && g_obd.vinLen < 17; i++) {
                char c = (char)src[i];
                if (c >= 32 && c <= 126) g_obd.vin[g_obd.vinLen++] = c;
              }
              g_obd.vin[g_obd.vinLen] = '\0';
            } else if (pid == 0x0A) {
              uint8_t dataCount = m.data_length_code - 3;
              const uint8_t* src = &m.data[3];
              if (dataCount > 0 && (src[0] & 0xF0)) { src++; dataCount--; }
              for (uint8_t i = 0; i < dataCount && g_obd.ecuLen < 32; i++) {
                char c = (char)src[i];
                if (c >= 32 && c <= 126) g_obd.ecuName[g_obd.ecuLen++] = c;
              }
              g_obd.ecuName[g_obd.ecuLen] = '\0';
            }
          }
        }
        xSemaphoreGive(g_obdMutex);
      }
    } else {
      // Check TWAI alerts and recover from bus-off or error-passive
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, pdMS_TO_TICKS(1)) == ESP_OK) {
        if (alerts & (TWAI_ALERT_BUS_OFF)) {
          twai_initiate_recovery();
        }
        if (alerts & (TWAI_ALERT_BUS_RECOVERED)) {
          // Ready to resume normal operation
        }
      }
    }
    // turn off activity LED after 50ms of inactivity
    if (millis() - g_last_can_rx_ms > 50) {
      digitalWrite(SHIELD_LED_BLUE_PIN, LOW);
    }
    vTaskDelay(1);
  }
}

// ===================== CAN Init =====================
static bool map_bitrate_to_timing(uint32_t bitrate, twai_timing_config_t* out_cfg) {
  switch (bitrate) {
    case 1000000: *out_cfg = TWAI_TIMING_CONFIG_1MBITS(); return true;
    case 800000:  *out_cfg = TWAI_TIMING_CONFIG_800KBITS(); return true;
    case 500000:  *out_cfg = TWAI_TIMING_CONFIG_500KBITS(); return true;
    case 250000:  *out_cfg = TWAI_TIMING_CONFIG_250KBITS(); return true;
    case 125000:  *out_cfg = TWAI_TIMING_CONFIG_125KBITS(); return true;
    case 100000:  *out_cfg = TWAI_TIMING_CONFIG_100KBITS(); return true;
    case 50000:   *out_cfg = TWAI_TIMING_CONFIG_50KBITS();  return true;
    case 25000:   *out_cfg = TWAI_TIMING_CONFIG_25KBITS();  return true;
    default: return false;
  }
}

static bool start_can_with_timing(const twai_timing_config_t& t_config) {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)SHIELD_CAN_TX, (gpio_num_t)SHIELD_CAN_RX, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = CAN_RX_QUEUE_LEN;
  g_config.tx_queue_len = CAN_TX_QUEUE_LEN;
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;
  twai_reconfigure_alerts(alerts, nullptr);
  return true;
}

static bool start_can_listen_only(const twai_timing_config_t& t_config) {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)SHIELD_CAN_TX, (gpio_num_t)SHIELD_CAN_RX, TWAI_MODE_LISTEN_ONLY);
  g_config.rx_queue_len = CAN_RX_QUEUE_LEN;
  g_config.tx_queue_len = 0; // not used in listen-only
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;
  twai_reconfigure_alerts(alerts, nullptr);
  return true;
}

static bool autodetect_can_bitrate(uint32_t* out_bps) {
  const uint32_t candidates[] = { 500000, 250000, 125000, 100000, 1000000 };
  int bestCount = -1; uint32_t bestBps = 0;
  for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); ++i) {
    twai_timing_config_t tcfg;
    if (!map_bitrate_to_timing(candidates[i], &tcfg)) continue;
    // stop any previous instance (ignore errors if not running)
    twai_stop(); twai_driver_uninstall();
    if (!start_can_listen_only(tcfg)) continue;
    const uint32_t startMs = millis();
    int rxCount = 0;
    while (millis() - startMs < 400) {
      twai_message_t m;
      if (twai_receive(&m, pdMS_TO_TICKS(20)) == ESP_OK) {
        rxCount++;
        if (rxCount >= 4) break; // enough evidence
      }
    }
    twai_stop(); twai_driver_uninstall();
    if (rxCount > bestCount) { bestCount = rxCount; bestBps = candidates[i]; }
    if (rxCount >= 4) break;
  }
  if (bestCount > 0) { *out_bps = bestBps; return true; }
  return false;
}

static bool init_can() {
  twai_timing_config_t t_config;
  // Try configured bitrate first; if no frames in listen-only autodetect safely
  if (!map_bitrate_to_timing(g_current_bitrate, &t_config)) t_config = TWAI_TIMING_CONFIG_500KBITS();
  // Optional safe autodetect step
  uint32_t detected = 0;
  if (autodetect_can_bitrate(&detected)) {
    g_current_bitrate = detected;
    map_bitrate_to_timing(g_current_bitrate, &t_config);
  }
  bool ok = start_can_with_timing(t_config);
  if (ok) g_can_running = true; else g_can_running = false;
  return ok;
}




static bool reconfigure_can_bitrate(uint32_t new_bitrate) {
  twai_timing_config_t t_config;
  if (!map_bitrate_to_timing(new_bitrate, &t_config)) return false;
  g_can_running = false;
  twai_stop();
  twai_driver_uninstall();
  bool ok = start_can_with_timing(t_config);
  if (ok) { g_current_bitrate = new_bitrate; g_can_running = true; }
  return ok;
}

// ===================== Flusher Task =====================
// no flusher task – client polls /poll

// ===================== HTTP server setup =====================
static void start_webserver() {
  g_server.on("/", HTTP_GET, root_get_handler);
  g_server.on("/poll", HTTP_GET, poll_get_handler);
  g_server.on("/stats", HTTP_GET, [](){
    // Check for stale indicators (no traffic for 2 seconds)
    uint32_t now = millis();
    if (now - g_last_can_traffic_ms > 2000) g_can_traffic_detected = false;
    if (now - g_last_obd_response_ms > 2000) g_obd_responses_detected = false;
    
    String out = "{\"rx_fps\":" + String(g_rx_fps) + 
                 ",\"tx_fps\":" + String(g_tx_fps) + 
                 ",\"can_bps\":" + String(g_current_bitrate) + 
                 ",\"can_traffic\":" + String(g_can_traffic_detected ? "true" : "false") + 
                 ",\"obd_responses\":" + String(g_obd_responses_detected ? "true" : "false") + 
                 ",\"tx_safe\":" + String(can_tx_safely() ? "true" : "false") + 
                 ",\"can_errors\":\"" + String(g_can_running ? (g_tx_write_blocked? "TX blocked" : "") : "CAN not running") + "\"," +
                 "\"tx_blocked\":" + String(g_tx_write_blocked ? "true" : "false") +
                 ",\"tx_suppressed\":" + String(g_tx_suppressed ? "true" : "false") +
                 "}";
    g_server.send(200, "application/json", out);
  });
  g_server.on("/tx", HTTP_POST, [](){
    String body = g_server.arg("plain");
    if (!body.length()) { g_server.send(408, "text/plain", "no body\n"); return; }
    std::unique_ptr<char[]> buf(new char[body.length() + 1]);
    memcpy(buf.get(), body.c_str(), body.length() + 1);
    char* cmd = buf.get();
    if (strncmp(cmd, "TX ", 3) == 0) {
      char* p = cmd + 3;
      uint32_t id = (uint32_t) strtoul(p, &p, 10);
      while (*p == ' ') p++;
      uint32_t ext = (uint32_t) strtoul(p, &p, 10);
      while (*p == ' ') p++;
      uint32_t dlc = (uint32_t) strtoul(p, &p, 10);
      while (*p == ' ') p++;
      uint8_t data[8] = {0};
      for (uint32_t i = 0; i < 8 && p && *p; i++) {
        char hex[3] = { '0', '0', 0 };
        if (*p) { hex[0] = *p++; }
        if (*p) { hex[1] = *p++; }
        data[i] = (uint8_t) strtoul(hex, nullptr, 16);
      }
      twai_message_t m = {};
      m.identifier = id & 0x1FFFFFFF;
      m.extd = ext ? 1 : 0;
      m.rtr = 0;
      m.data_length_code = (uint8_t) (dlc & 0x0F);
      memcpy(m.data, data, 8);
      esp_err_t txr = twai_transmit(&m, pdMS_TO_TICKS(5));
      if (txr == ESP_OK) {
        g_tx_count++;
        g_tx_fail_streak = 0;
      } else {
        if (g_tx_fail_streak < 255) g_tx_fail_streak++;
        if (g_tx_fail_streak >= 3) g_tx_write_blocked = true; // likely listen-only
      }
    }
    g_server.send(200, "text/plain", "OK\n");
  });
  g_server.on("/speed", HTTP_POST, [](){
    String body = g_server.arg("plain");
    if (!body.length()) { g_server.send(400, "text/plain", "missing body\n"); return; }
    const char* c = body.c_str();
    if (strncmp(c, "SPEED ", 7) == 0) c += 7;
    // Skip non-digits to be robust against prefixes/whitespace
    while (*c && (*c < '0' || *c > '9')) c++;
    uint32_t spd = (uint32_t) strtoul(c, nullptr, 10);
    if (spd == 0) { g_server.send(400, "text/plain", "bad speed\n"); return; }
    bool ok = reconfigure_can_bitrate(spd);
    if (ok) { g_prefs.putUInt("can_bps", spd); }
    if (ok) g_server.send(200, "text/plain", "OK\n"); else g_server.send(500, "text/plain", "FAIL\n");
  });
  // OBD endpoints
  g_server.on("/obd", HTTP_GET, [](){ g_server.sendHeader("Cache-Control","no-store"); g_server.send_P(200, "text/html", OBD_HTML); });
  // Single message monitor page
  g_server.on("/msg", HTTP_GET, [](){ g_server.sendHeader("Cache-Control","no-store"); g_server.send_P(200, "text/html", MSG_HTML); });
  g_server.on("/favicon.ico", HTTP_GET, [](){
    // Serve the same small 16x16 icon embedded earlier by redirecting to the data URL present in main page.
    // For simplicity we just return a 1x1 transparent PNG; header logo still uses the data URL.
    static const uint8_t PNG1x1[] = {0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A,0x00,0x00,0x00,0x0D,0x49,0x48,0x44,0x52,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x08,0x06,0x00,0x00,0x00,0x1F,0x15,0xC4,0x89,0x00,0x00,0x00,0x0A,0x49,0x44,0x41,0x54,0x78,0x9C,0x63,0x00,0x01,0x00,0x00,0x05,0x00,0x01,0x0D,0x0A,0x2D,0xB4,0x00,0x00,0x00,0x00,0x49,0x45,0x4E,0x44,0xAE,0x42,0x60,0x82};
    g_server.sendHeader("Cache-Control", "max-age=86400");
    g_server.setContentLength(sizeof(PNG1x1));
    g_server.send(200, "image/png", "");
    WiFiClient c = g_server.client(); if (c) c.write(PNG1x1, sizeof(PNG1x1));
  });
  // Message page favicon reuse
  g_server.on("/obd_poll", HTTP_GET, [](){
    String out;
    out.reserve(512);
    bool enabled=false; float rpm=0,speed=0,fuel=0,thr=0,load=0,mapkpa=0,iat=0,maf=0,timing=0,baro=0,amb=0,evap=0; int cool=0; uint32_t runtime=0; uint8_t dtcN=0; char dtcLoc[5][8] = {};
    uint32_t pid00Mask=0; bool pid00Valid=false;
    if (xSemaphoreTake(g_obdMutex, 10) == pdTRUE) {
      enabled = g_obd.enabled; load=g_obd.engineLoadPct; rpm=g_obd.rpm; speed=g_obd.speedKph; fuel=g_obd.fuelPct; thr=g_obd.throttlePct; cool=g_obd.coolantC; mapkpa=g_obd.intakeMapKpa; iat=g_obd.intakeAirTempC; maf=g_obd.mafGps; timing=g_obd.timingAdvanceDeg; baro=g_obd.baroKpa; amb=g_obd.ambientAirTempC; runtime=g_obd.runtimeSec; evap=g_obd.evapPurgePct; dtcN=g_obd.dtcCount; for (uint8_t i=0;i<dtcN && i<5;i++){ strncpy(dtcLoc[i], g_obd.dtc[i], sizeof(dtcLoc[i])-1); }
      pid00Mask = g_obd.pid00Mask; pid00Valid = g_obd.pid00Valid;
      xSemaphoreGive(g_obdMutex);
    }
    out += '{';
    out += "\"enabled\":"; out += (enabled?"true":"false");
    out += ",\"load_pct\":"; out += String(load,0);
    out += ",\"rpm\":"; out += String(rpm,0);
    out += ",\"speed_kph\":"; out += String(speed,0);
    out += ",\"fuel_pct\":"; out += String(fuel,0);
    out += ",\"throttle_pct\":"; out += String(thr,0);
    out += ",\"coolant_c\":"; out += String(cool);
    out += ",\"map_kpa\":"; out += String(mapkpa,0);
    out += ",\"iat_c\":"; out += String(iat,0);
    out += ",\"maf_gps\":"; out += String(maf,1);
    out += ",\"timing_adv_deg\":"; out += String(timing,1);
    out += ",\"baro_kpa\":"; out += String(baro,0);
    out += ",\"amb_c\":"; out += String(amb,0);
    out += ",\"runtime_sec\":"; out += String(runtime);
    out += ",\"evap_purge_pct\":"; out += String(evap,0);
    out += ",\"module_voltage_v\":"; out += String(g_obd.moduleVoltageV,2);
    out += ",\"obd_std\":"; out += String(g_obd.obdStdCode);
    out += ",\"vin\":\""; out += String(g_obd.vin); out += "\"";
    out += ",\"ecu_name\":\""; out += String(g_obd.ecuName); out += "\"";
    // Supported PIDs list (01-20)
    out += ",\"pid00_supported\":[";
    if (pid00Valid) {
      bool first=true;
      for (int pid = 1; pid <= 32; pid++) {
        int bit = 32 - pid; // bit31 -> PID 01, bit0 -> PID 20
        if (pid00Mask & (1UL << bit)) {
          if (!first) out += ','; first=false;
          char buf[5]; snprintf(buf, sizeof(buf), "\"%02X\"", pid);
          out += buf;
        }
      }
    }
    out += "]";
    out += ",\"dtc\":[";
    for (uint8_t i=0;i<dtcN;i++) { if (i) out += ','; out += '\"'; out += dtcLoc[i]; out += '\"'; }
    out += "]}";
    g_server.send(200, "application/json", out);
  });
  // Return one frame (latest) by id/ext
  g_server.on("/msg_poll", HTTP_GET, [](){
    String sid = g_server.arg("id");
    String sext = g_server.arg("ext");
    if (!sid.length()) { g_server.send(400, "application/json", "{}\n"); return; }
    uint32_t id = (uint32_t) strtoul(sid.c_str(), nullptr, 10);
    uint8_t ext = (uint8_t)(sext.length() ? (uint32_t)strtoul(sext.c_str(), nullptr, 10) : 0);
    uint32_t key = make_key(id, ext);
    FrameEntry copy = {};
    bool found = false;
    if (xSemaphoreTake(g_tableMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      FrameEntry* e = table_find_by_key(key);
      if (e) { memcpy(&copy, e, sizeof(copy)); found = true; }
      xSemaphoreGive(g_tableMutex);
    }
    String out = "{";
    if (found) {
      out += "\"id\":" + String(copy.id);
      out += ",\"ext\":" + String(copy.ext);
      out += ",\"dlc\":" + String(copy.dlc);
      out += ",\"count\":" + String(copy.count);
      out += ",\"age_ms\":" + String((uint32_t)(millis() - copy.last_ms));
      out += ",\"data\":[";
      for (int i=0;i<8;i++){ if(i) out+=","; char buf[8]; snprintf(buf,sizeof(buf),"%u", copy.data[i]); out += buf; }
      out += "]";
    }
    out += "}";
    g_server.send(200, "application/json", out);
  });
  g_server.on("/obd_toggle", HTTP_POST, [](){
    String body = g_server.arg("plain");
    bool enable = body.startsWith("ENABLE");
    if (xSemaphoreTake(g_obdMutex, portMAX_DELAY) == pdTRUE) {
      g_obd.enabled = enable;
      g_obd.lastPollMs = 0;
      xSemaphoreGive(g_obdMutex);
    }
    g_server.send(200, "text/plain", "OK\n");
  });
  // Control whether we actively transmit OBD requests (separate from parsing)
  g_server.on("/obd_tx", HTTP_POST, [](){
    String body = g_server.arg("plain");
    if (body.startsWith("ON")) {
      g_obd_tx_enabled = true;
    } else if (body.startsWith("OFF")) {
      g_obd_tx_enabled = false;
    }
    g_server.send(200, "text/plain", "OK\n");
  });

  g_server.begin();
}


// ===================== WiFi AP Init =====================
static void init_ap() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  IPAddress IP = WiFi.softAPIP();
  LOGF("AP IP address: %s\n", IP.toString().c_str());
  pinMode(SHIELD_LED_GREEN_PIN, OUTPUT);
  digitalWrite(SHIELD_LED_GREEN_PIN, HIGH); // server live
  pinMode(SHIELD_LED_BLUE_PIN, OUTPUT);
  digitalWrite(SHIELD_LED_BLUE_PIN, LOW);
}

// ===================== Arduino setup/loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);
  LOGLN("\MrDIY Live CAN and OBDII starting ...");

  // Initialize AP first - most critical for connectivity
  init_ap();

  // Preferences for persistence
  g_prefs.begin("candash", false);
  uint32_t saved_bps = g_prefs.getUInt("can_bps", 0);
  if (saved_bps != 0) g_current_bitrate = saved_bps;

  // Allocate aggregator table in PSRAM if available, fallback to DRAM
  #if defined(BOARD_HAS_PSRAM)
    if (psramFound()) {
      g_table = (FrameEntry*) heap_caps_malloc(sizeof(FrameEntry) * AGG_TABLE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
  #endif
  if (!g_table) {
    g_table = (FrameEntry*) heap_caps_malloc(sizeof(FrameEntry) * AGG_TABLE_SIZE, MALLOC_CAP_8BIT);
  }
  if (!g_table) {
    LOGLN("Failed to allocate aggregator table");
    abort();
  }
  memset(g_table, 0, sizeof(FrameEntry) * AGG_TABLE_SIZE);

  g_tableMutex = xSemaphoreCreateMutex();
  if (!g_tableMutex) {
    LOGLN("Failed to create mutex");
    abort();
  }

  // OBD state
  memset(&g_obd, 0, sizeof(g_obd));
  g_obdMutex = xSemaphoreCreateMutex();

  start_webserver();

  if (!init_can()) {
    LOGLN("CAN init failed. Restarting in 5s ...");
    delay(5000);
    ESP.restart();
  }

  // Tasks
  xTaskCreatePinnedToCore(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
  // Lightweight OBD polling will piggyback inside loop() to avoid extra task
}

void loop() {
  g_server.handleClient();
  
  // Calculate FPS every second
  uint32_t now = millis();
  if (now - g_last_stats_ms >= 1000) {
    g_rx_fps = g_rx_count;
    g_tx_fps = g_tx_count;
    g_rx_count = 0;
    g_tx_count = 0;
    g_last_stats_ms = now;
  }
  
  // Simple OBD poller: send one PID every interval when enabled and CAN running
  static uint32_t last_sent = 0;
  static size_t idx = 0;
  if (g_can_running && g_obd.enabled && g_obd_tx_enabled && now - last_sent >= g_obd_poll_interval_ms) {
    // Safety: only TX OBD when we see recent RX traffic (bitrate confirmed)
    if (can_tx_safely() && !g_tx_suppressed && !g_tx_write_blocked) {
      // Interleave Mode 09 requests occasionally to gather vehicle info
      static uint8_t mode9Phase = 0; // 0..3
      if (mode9Phase == 0) {
        send_obd_mode09_request(0x02); // VIN
      } else if (mode9Phase == 2) {
        send_obd_mode09_request(0x0A); // ECU name
      } else {
        send_obd_pid_request(OBD_PIDS[idx]);
        idx = (idx + 1) % NUM_OBD_PIDS;
      }
      mode9Phase = (mode9Phase + 1) & 3; // cycle 0..3
    } else {
      // If not safe, skip TX for this tick; UI will show no OBD responses and can_errors already
    }
    last_sent = now;
  }

  delay(1);
}
 
