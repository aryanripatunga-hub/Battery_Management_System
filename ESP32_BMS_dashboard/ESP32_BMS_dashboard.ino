#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#define WIFI_SSID      "Aryank"
#define WIFI_PASSWORD  "aryan*12#"

/* ── Serial2 pins to STM32  */
#define STM32_RX  16   
#define STM32_TX  17   
#define STM32_BAUD 115200

/* ── Server objects  */
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

/* ── Line buffer for STM32 serial framing  */
#define LINE_BUF 400
static char line_buf[LINE_BUF];
static int  line_pos = 0;

static String lastJson = "{}";
static const char PAGE[] PROGMEM = R"====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>3S BMS Monitor</title>
<style>
/* ── Google Fonts ── */
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&family=Inter:wght@400;500;600&display=swap');

/* ── Tokens ── */
:root{
  --chassis:  #0D0F0D;
  --panel:    #111411;
  --panel2:   #181C18;
  --border:   #232823;
  --border2:  #2E352E;
  --phosphor: #39FF14;
  --p-dim:    #1E8A0A;
  --p-dark:   #0C3A05;
  --amber:    #FFB900;
  --amber-d:  #7A5800;
  --red:      #FF3B30;
  --red-d:    #5C1510;
  --blue:     #0A84FF;
  --text:     #C8D4C8;
  --muted:    #556655;
  --font-data:'JetBrains Mono',monospace;
  --font-ui:  'Inter',system-ui,sans-serif;
  --r:        2px;
}

*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}

body{
  background:var(--chassis);
  color:var(--text);
  font-family:var(--font-ui);
  font-size:14px;
  line-height:1.5;
  min-height:100vh;
}

/* ── HEADER ── */
header{
  background:var(--panel);
  border-bottom:1px solid var(--border);
  display:flex;
  align-items:center;
  justify-content:space-between;
  padding:0 24px;
  height:52px;
  position:sticky;top:0;z-index:200;
}
.hdr-left{display:flex;align-items:center;gap:14px}
.hdr-logo{
  font-family:var(--font-data);
  font-weight:700;font-size:15px;
  color:var(--phosphor);
  letter-spacing:.12em;
  text-transform:uppercase;
}
.hdr-sub{
  font-size:10px;color:var(--muted);
  letter-spacing:.18em;text-transform:uppercase;
  padding-top:2px;
}
.hdr-right{display:flex;align-items:center;gap:16px}

/* connection pill */
#pill{
  display:flex;align-items:center;gap:7px;
  padding:5px 12px;
  background:var(--panel2);
  border:1px solid var(--border);
  font-family:var(--font-data);font-size:11px;
  border-radius:20px;
}
#pdot{
  width:8px;height:8px;border-radius:50%;
  background:var(--muted);
  transition:background .4s,box-shadow .4s;
}
#pdot.live{background:var(--phosphor);box-shadow:0 0 8px var(--phosphor)}
#pdot.err {background:var(--red);}
#ptxt{color:var(--text)}

/* timestamp */
#ts{
  font-family:var(--font-data);font-size:10px;
  color:var(--muted);
}

/* ── ALERT STRIP ── */
#alerts{
  display:none;
  padding:0 24px;
  gap:8px;
  flex-direction:column;
  margin-top:16px;
}
#alerts.show{display:flex}
.alert{
  display:flex;align-items:flex-start;gap:10px;
  padding:12px 16px;
  border-left:3px solid var(--red);
  background:rgba(255,59,48,.08);
  font-size:13px;
  border-radius:0 var(--r) var(--r) 0;
}
.alert-icon{font-size:16px;flex-shrink:0;margin-top:1px}
.alert-body strong{color:var(--red);font-size:13px}
.alert-body p{color:#FF8A83;font-size:12px;margin-top:2px}

/* ── GRID ── */
.grid{
  display:grid;
  grid-template-columns:repeat(12,1fr);
  gap:12px;
  padding:16px 24px 32px;
  max-width:1440px;
  margin:0 auto;
}

/* ── CARD ── */
.card{
  background:var(--panel);
  border:1px solid var(--border);
  padding:16px;
  position:relative;
}
.card::before{
  content:'';
  position:absolute;top:0;left:0;right:0;height:1px;
  background:linear-gradient(90deg,var(--phosphor) 0%,transparent 60%);
  opacity:.25;
}
.card-title{
  font-family:var(--font-ui);
  font-size:9px;font-weight:600;
  letter-spacing:.18em;text-transform:uppercase;
  color:var(--muted);
  margin-bottom:14px;
}

/* span helpers */
.span-12{grid-column:span 12}
.span-8 {grid-column:span 8}
.span-6 {grid-column:span 6}
.span-4 {grid-column:span 4}
.span-3 {grid-column:span 3}
.span-2 {grid-column:span 2}

/* ── PACK ROW ── */
.pack-row{
  display:grid;
  grid-template-columns:repeat(5,1fr);
  gap:8px;
}
.pstat{
  background:var(--panel2);
  border:1px solid var(--border);
  padding:14px 12px;
  text-align:center;
}
.pstat-val{
  font-family:var(--font-data);
  font-size:28px;font-weight:700;
  color:var(--phosphor);
  line-height:1;
  letter-spacing:-.01em;
}
.pstat-unit{
  font-family:var(--font-data);
  font-size:11px;color:var(--muted);
  margin-top:4px;
}
.pstat-lbl{
  font-size:9px;font-weight:600;
  letter-spacing:.15em;text-transform:uppercase;
  color:var(--muted);margin-top:6px;
}

/* SOC bar */
.soc-track{
  height:6px;
  background:var(--panel2);
  border:1px solid var(--border);
  margin-top:14px;
  overflow:hidden;
}
.soc-fill{
  height:100%;
  background:linear-gradient(90deg,var(--p-dim),var(--phosphor));
  transition:width .5s ease;
  width:0%;
}

/* ── CELL CARD ── */
.cell-card{
  grid-column:span 4;
}
.cell-hdr{
  display:flex;align-items:center;justify-content:space-between;
  margin-bottom:14px;
}
.cell-id{
  font-family:var(--font-data);
  font-size:13px;font-weight:700;
  color:var(--phosphor);
  letter-spacing:.1em;
}
.bal-tag{
  font-size:9px;font-weight:600;
  letter-spacing:.12em;text-transform:uppercase;
  padding:3px 8px;
  background:var(--amber-d);
  border:1px solid var(--amber);
  color:var(--amber);
  opacity:0;
  transition:opacity .3s;
}
.bal-tag.on{opacity:1}

/* data rows */
.drow{
  display:flex;align-items:baseline;
  justify-content:space-between;
  padding:8px 0;
  border-bottom:1px solid var(--border);
}
.drow:last-of-type{border-bottom:none}
.dlbl{
  font-size:11px;color:var(--muted);
  letter-spacing:.04em;
}
.dval{
  font-family:var(--font-data);
  font-size:15px;font-weight:600;
  color:var(--text);
}
.dval.hi{color:var(--phosphor)}

/* mini progress */
.mini-bar{
  height:3px;background:var(--panel2);
  border:1px solid var(--border);
  margin-top:4px;overflow:hidden;
}
.mini-fill{height:100%;background:var(--p-dim);transition:width .5s}
.mini-fill.soh{background:var(--blue)}

/* canvas trace */
.trace-wrap{margin-top:12px}
.trace-lbl{
  font-size:9px;color:var(--muted);
  letter-spacing:.1em;text-transform:uppercase;
  margin-bottom:4px;
}
canvas.trace{
  width:100%;height:56px;display:block;
  background:var(--panel2);
  border:1px solid var(--border);
}

/* ── PROTECTION CARD ── */
.prot-grid{
  display:grid;grid-template-columns:repeat(3,1fr);gap:8px;
  margin-bottom:12px;
}
.pfault{
  padding:14px 10px;text-align:center;
  background:var(--panel2);
  border:1px solid var(--border);
  transition:background .3s,border-color .3s;
}
.pfault-name{
  font-size:9px;font-weight:600;
  letter-spacing:.15em;text-transform:uppercase;
  color:var(--muted);margin-bottom:8px;
}
.pfault-state{
  font-family:var(--font-data);
  font-size:14px;font-weight:700;
}
.pfault.ok   .pfault-state{color:var(--phosphor)}
.pfault.trip{
  background:rgba(255,59,48,.1);
  border-color:rgba(255,59,48,.5);
}
.pfault.trip .pfault-state{color:var(--red)}

/* FET status */
.fet-row{
  display:flex;align-items:center;gap:10px;
  padding:10px 14px;
  background:var(--panel2);
  border:1px solid var(--border);
  margin-top:8px;
}
.fet-led{
  width:12px;height:12px;border-radius:50%;
  background:var(--muted);flex-shrink:0;
  transition:background .3s,box-shadow .3s;
}
.fet-led.on {background:var(--phosphor);box-shadow:0 0 10px var(--phosphor)}
.fet-led.off{background:var(--red);     box-shadow:0 0 10px var(--red)}
.fet-lbl{flex:1;font-size:12px;color:var(--text)}
.fet-val{
  font-family:var(--font-data);font-size:13px;font-weight:700;
}
.fet-val.on {color:var(--phosphor)}
.fet-val.off{color:var(--red)}

/* ── SPARKLINE CARD ── */
canvas#pack-trace{
  width:100%;height:100px;display:block;
  background:var(--panel2);
  border:1px solid var(--border);
  margin-top:8px;
}
.trace-legend{
  display:flex;gap:16px;margin-top:8px;
  font-size:10px;color:var(--muted);
}
.trace-legend span{display:flex;align-items:center;gap:5px}
.dot{width:8px;height:8px;border-radius:50%;flex-shrink:0}

/* ── COMMAND CARD ── */
.cmd-grid{
  display:grid;grid-template-columns:1fr 1fr;gap:8px;
}
.cmd-btn{
  padding:10px;
  background:var(--panel2);
  border:1px solid var(--border);
  color:var(--text);
  font-family:var(--font-ui);
  font-size:12px;font-weight:600;
  cursor:pointer;
  transition:background .15s,border-color .15s;
  border-radius:var(--r);
}
.cmd-btn:hover{background:var(--border);border-color:var(--border2)}
.cmd-btn.danger:hover{
  background:rgba(255,59,48,.12);
  border-color:rgba(255,59,48,.4);
  color:var(--red);
}
.cmd-btn.safe:hover{
  background:rgba(57,255,20,.08);
  border-color:rgba(57,255,20,.3);
  color:var(--phosphor);
}
#cmd-out{
  margin-top:10px;
  padding:8px;
  background:var(--panel2);
  border:1px solid var(--border);
  font-family:var(--font-data);font-size:10px;
  color:var(--muted);
  min-height:32px;
}

/* ── SOH TABLE ── */
.soh-table{width:100%;border-collapse:collapse;font-family:var(--font-data)}
.soh-table th{
  font-family:var(--font-ui);
  font-size:9px;font-weight:600;letter-spacing:.15em;text-transform:uppercase;
  color:var(--muted);padding:6px 8px;
  border-bottom:1px solid var(--border);text-align:left;
}
.soh-table td{
  padding:8px;font-size:13px;
  border-bottom:1px solid var(--border);
  color:var(--text);
}
.soh-table td:first-child{color:var(--phosphor);font-weight:700}

/* ── RESPONSIVE ── */
@media(max-width:900px){
  .pack-row{grid-template-columns:repeat(3,1fr)}
  .cell-card{grid-column:span 6}
  .span-4{grid-column:span 6}
  .span-8{grid-column:span 12}
}
@media(max-width:600px){
  header{padding:0 14px}
  .grid{padding:12px 14px 24px;gap:10px}
  .pack-row{grid-template-columns:1fr 1fr}
  .cell-card,.span-4,.span-6,.span-8{grid-column:span 12}
  .prot-grid{grid-template-columns:1fr 1fr}
  .pstat-val{font-size:22px}
}
</style>
</head>
<body>

<!-- ══ HEADER ══════════════════════════════════════════════════════════ -->
<header>
  <div class="hdr-left">
    <div>
      <div class="hdr-logo">⚡ BMS Monitor</div>
      <div class="hdr-sub">3S Li-Ion Pack · STM32 F446RE</div>
    </div>
  </div>
  <div class="hdr-right">
    <div id="ts">--:--:--</div>
    <div id="pill">
      <div id="pdot"></div>
      <span id="ptxt">Connecting…</span>
    </div>
  </div>
</header>

<!-- ══ ALERT STRIP ══════════════════════════════════════════════════════ -->
<div id="alerts"></div>

<!-- ══ MAIN GRID ════════════════════════════════════════════════════════ -->
<div class="grid">

  <!-- ── Pack Overview ─────────────────────────────── span 12 ── -->
  <div class="card span-12">
    <div class="card-title">Pack Overview</div>
    <div class="pack-row">
      <div class="pstat">
        <div class="pstat-val" id="pv-soc">--</div>
        <div class="pstat-unit">%</div>
        <div class="pstat-lbl">Pack SOC</div>
      </div>
      <div class="pstat">
        <div class="pstat-val" id="pv-soh">--</div>
        <div class="pstat-unit">%</div>
        <div class="pstat-lbl">Pack SOH</div>
      </div>
      <div class="pstat">
        <div class="pstat-val" id="pv-cur">--</div>
        <div class="pstat-unit">A</div>
        <div class="pstat-lbl">Current</div>
      </div>
      <div class="pstat">
        <div class="pstat-val" id="pv-pw">--</div>
        <div class="pstat-unit">W</div>
        <div class="pstat-lbl">Power</div>
      </div>
      <div class="pstat">
        <div class="pstat-val" id="pv-bv">--</div>
        <div class="pstat-unit">V</div>
        <div class="pstat-lbl">Bus Voltage</div>
      </div>
    </div>
    <div class="soc-track">
      <div class="soc-fill" id="bar-pack-soc"></div>
    </div>
  </div>

  <!-- ── Cell 1 ────────────────────────────────────── span 4  ── -->
  <div class="card cell-card" id="cc1">
    <div class="cell-hdr">
      <span class="cell-id">CELL 1</span>
      <span class="bal-tag" id="bt1">BALANCING</span>
    </div>
    <div class="drow">
      <span class="dlbl">Voltage</span>
      <span class="dval hi" id="c1v">-- V</span>
    </div>
    <div class="drow">
      <span class="dlbl">Temperature</span>
      <span class="dval" id="c1t">-- °C</span>
    </div>
    <div class="drow">
      <span class="dlbl">SOC</span>
      <span class="dval" id="c1s">--%</span>
    </div>
    <div class="mini-bar"><div class="mini-fill" id="mb1s"></div></div>
    <div class="drow" style="margin-top:8px">
      <span class="dlbl">SOH</span>
      <span class="dval" id="c1h">--%</span>
    </div>
    <div class="mini-bar"><div class="mini-fill soh" id="mb1h"></div></div>
    <div class="drow" style="margin-top:8px">
      <span class="dlbl">Internal Ri</span>
      <span class="dval" id="c1r">-- mΩ</span>
    </div>
    <div class="trace-wrap">
      <div class="trace-lbl">Voltage trace</div>
      <canvas class="trace" id="cv1"></canvas>
    </div>
  </div>

  <!-- ── Cell 2 ────────────────────────────────────── span 4  ── -->
  <div class="card cell-card" id="cc2">
    <div class="cell-hdr">
      <span class="cell-id">CELL 2</span>
      <span class="bal-tag" id="bt2">BALANCING</span>
    </div>
    <div class="drow">
      <span class="dlbl">Voltage</span>
      <span class="dval hi" id="c2v">-- V</span>
    </div>
    <div class="drow">
      <span class="dlbl">Temperature</span>
      <span class="dval" id="c2t">-- °C</span>
    </div>
    <div class="drow">
      <span class="dlbl">SOC</span>
      <span class="dval" id="c2s">--%</span>
    </div>
    <div class="mini-bar"><div class="mini-fill" id="mb2s"></div></div>
    <div class="drow" style="margin-top:8px">
      <span class="dlbl">SOH</span>
      <span class="dval" id="c2h">--%</span>
    </div>
    <div class="mini-bar"><div class="mini-fill soh" id="mb2h"></div></div>
    <div class="drow" style="margin-top:8px">
      <span class="dlbl">Internal Ri</span>
      <span class="dval" id="c2r">-- mΩ</span>
    </div>
    <div class="trace-wrap">
      <div class="trace-lbl">Voltage trace</div>
      <canvas class="trace" id="cv2"></canvas>
    </div>
  </div>

  <!-- ── Cell 3 ────────────────────────────────────── span 4  ── -->
  <div class="card cell-card" id="cc3">
    <div class="cell-hdr">
      <span class="cell-id">CELL 3</span>
      <span class="bal-tag" id="bt3">BALANCING</span>
    </div>
    <div class="drow">
      <span class="dlbl">Voltage</span>
      <span class="dval hi" id="c3v">-- V</span>
    </div>
    <div class="drow">
      <span class="dlbl">Temperature</span>
      <span class="dval" id="c3t">-- °C</span>
    </div>
    <div class="drow">
      <span class="dlbl">SOC</span>
      <span class="dval" id="c3s">--%</span>
    </div>
    <div class="mini-bar"><div class="mini-fill" id="mb3s"></div></div>
    <div class="drow" style="margin-top:8px">
      <span class="dlbl">SOH</span>
      <span class="dval" id="c3h">--%</span>
    </div>
    <div class="mini-bar"><div class="mini-fill soh" id="mb3h"></div></div>
    <div class="drow" style="margin-top:8px">
      <span class="dlbl">Internal Ri</span>
      <span class="dval" id="c3r">-- mΩ</span>
    </div>
    <div class="trace-wrap">
      <div class="trace-lbl">Voltage trace</div>
      <canvas class="trace" id="cv3"></canvas>
    </div>
  </div>

  <!-- ── Protection Status ─────────────────────────── span 4  ── -->
  <div class="card span-4">
    <div class="card-title">Protection Status</div>
    <div class="prot-grid">
      <div class="pfault ok" id="fi-uvp">
        <div class="pfault-name">UVP</div>
        <div class="pfault-state" id="fs-uvp">OK</div>
      </div>
      <div class="pfault ok" id="fi-ocp">
        <div class="pfault-name">OCP</div>
        <div class="pfault-state" id="fs-ocp">OK</div>
      </div>
      <div class="pfault ok" id="fi-otp">
        <div class="pfault-name">OTP</div>
        <div class="pfault-state" id="fs-otp">OK</div>
      </div>
    </div>
    <div class="fet-row">
      <div class="fet-led" id="fet-led"></div>
      <span class="fet-lbl">Load FET (PB1)</span>
      <span class="fet-val" id="fet-val">--</span>
    </div>
    <!-- Cell voltage imbalance -->
    <div style="margin-top:14px">
      <div class="card-title">Cell Spread</div>
      <div class="drow">
        <span class="dlbl">V<sub>max</sub> − V<sub>min</sub></span>
        <span class="dval" id="vspread">-- mV</span>
      </div>
      <div class="drow">
        <span class="dlbl">V<sub>max</sub></span>
        <span class="dval" id="vmax">-- V</span>
      </div>
      <div class="drow">
        <span class="dlbl">V<sub>min</sub></span>
        <span class="dval" id="vmin">-- V</span>
      </div>
    </div>
  </div>

  <!-- ── Pack Voltage History ──────────────────────── span 8  ── -->
  <div class="card span-8">
    <div class="card-title">Pack Voltage History — 60 s</div>
    <canvas id="pack-trace"></canvas>
    <div class="trace-legend">
      <span><span class="dot" style="background:#39FF14"></span>Cell 1</span>
      <span><span class="dot" style="background:#0A84FF"></span>Cell 2</span>
      <span><span class="dot" style="background:#FFB900"></span>Cell 3</span>
    </div>
  </div>

  <!-- ── SOH Table ─────────────────────────────────── span 4  ── -->
  <div class="card span-4">
    <div class="card-title">Health &amp; Resistance</div>
    <table class="soh-table">
      <thead>
        <tr>
          <th>Cell</th><th>SOH %</th><th>Ri (mΩ)</th><th>Status</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td>C1</td>
          <td id="st1h">--</td>
          <td id="st1r">--</td>
          <td id="st1s" style="color:var(--phosphor)">--</td>
        </tr>
        <tr>
          <td>C2</td>
          <td id="st2h">--</td>
          <td id="st2r">--</td>
          <td id="st2s" style="color:var(--phosphor)">--</td>
        </tr>
        <tr>
          <td>C3</td>
          <td id="st3h">--</td>
          <td id="st3r">--</td>
          <td id="st3s" style="color:var(--phosphor)">--</td>
        </tr>
        <tr>
          <td style="color:var(--amber);font-weight:700">PACK</td>
          <td id="stph">--</td>
          <td>--</td>
          <td id="stps" style="color:var(--phosphor)">--</td>
        </tr>
      </tbody>
    </table>
  </div>

  <!-- ── Self-Test Commands ────────────────────────── span 4  ── -->
  <div class="card span-4">
    <div class="card-title">Self-Test Commands</div>
    <div class="cmd-grid">
      <button class="cmd-btn danger" onclick="sendCmd('U')">⚡ Inject UVP</button>
      <button class="cmd-btn danger" onclick="sendCmd('O')">⚡ Inject OCP</button>
      <button class="cmd-btn danger" onclick="sendCmd('T')">🌡 Inject OTP</button>
      <button class="cmd-btn safe"   onclick="sendCmd('R')">✔ Reset Faults</button>
    </div>
    <div id="cmd-out">— ready —</div>
  </div>

  <!-- ── Temperature Card ──────────────────────────── span 4  ── -->
  <div class="card span-4">
    <div class="card-title">Thermal Status</div>
    <div class="drow">
      <span class="dlbl">Cell 1</span>
      <span class="dval" id="th1">-- °C</span>
    </div>
    <div class="drow">
      <span class="dlbl">Cell 2</span>
      <span class="dval" id="th2">-- °C</span>
    </div>
    <div class="drow">
      <span class="dlbl">Cell 3</span>
      <span class="dval" id="th3">-- °C</span>
    </div>
    <div class="drow" style="margin-top:6px">
      <span class="dlbl">OTP threshold</span>
      <span class="dval" style="color:var(--muted)">55.0 °C</span>
    </div>
    <div class="drow">
      <span class="dlbl">OTP recovery</span>
      <span class="dval" style="color:var(--muted)">45.0 °C</span>
    </div>
  </div>

</div><!-- /grid -->

<script>
/* ════════════════════════════════════════════════════════════════════════
   WebSocket client
   ════════════════════════════════════════════════════════════════════════ */
const pdot = document.getElementById('pdot');
const ptxt = document.getElementById('ptxt');
let ws, reconnT;

function connect(){
  ws = new WebSocket('ws://'+location.host+'/ws');
  ws.onopen  = ()=>{ pdot.className='live'; ptxt.textContent='Live'; clearTimeout(reconnT); };
  ws.onclose = ws.onerror = ()=>{
    pdot.className='err'; ptxt.textContent='Reconnecting…';
    reconnT = setTimeout(connect, 3000);
  };
  ws.onmessage = e=>{ try{ update(JSON.parse(e.data)); }catch(_){} };
}
connect();

function sendCmd(c){
  if(ws && ws.readyState===1){
    ws.send(c);
    document.getElementById('cmd-out').textContent =
      new Date().toLocaleTimeString()+' — sent: '+c;
  }
}

const MAX = 120;  /* 60 s at 2 Hz */
const hist = { v1:[], v2:[], v3:[] };

function pushHist(d){
  ['v1','v2','v3'].forEach(k=>{
    hist[k].push(d[k]);
    if(hist[k].length > MAX) hist[k].shift();
  });
}

function drawCellTrace(canvasId, dataArr, color){
  const el  = document.getElementById(canvasId);
  if(!el) return;
  const W   = el.clientWidth  || el.offsetWidth  || 260;
  const H   = el.clientHeight || el.offsetHeight || 56;
  const dpr = window.devicePixelRatio||1;
  el.width  = W*dpr; el.height = H*dpr;
  const ctx = el.getContext('2d');
  ctx.scale(dpr,dpr);
  ctx.clearRect(0,0,W,H);
  if(dataArr.length < 2) return;

  const mn = Math.min(...dataArr)-0.005;
  const mx = Math.max(...dataArr)+0.005;
  const rng= mx-mn||0.01;
  const n  = dataArr.length;

  /* grid lines */
  ctx.strokeStyle='#1E2A1E'; ctx.lineWidth=1;
  for(let row=1;row<4;row++){
    const y = (row/4)*H;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
  }

  /* trace */
  ctx.beginPath();
  ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.lineJoin='round';
  dataArr.forEach((v,i)=>{
    const x = (i/(n-1))*W;
    const y = H - ((v-mn)/rng)*(H-4) - 2;
    i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.stroke();

  /* glow */
  ctx.beginPath();
  ctx.strokeStyle=color; ctx.lineWidth=4;
  ctx.globalAlpha=0.12;
  dataArr.forEach((v,i)=>{
    const x=(i/(n-1))*W;
    const y=H-((v-mn)/rng)*(H-4)-2;
    i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
  });
  ctx.stroke(); ctx.globalAlpha=1;

  /* axis labels */
  ctx.fillStyle='#556655'; ctx.font='8px JetBrains Mono';
  ctx.textAlign='right';
  ctx.fillText(mx.toFixed(3)+'V', W-2, 9);
  ctx.fillText(mn.toFixed(3)+'V', W-2, H-2);
}

function drawPackTrace(){
  const el = document.getElementById('pack-trace');
  if(!el) return;
  const W  = el.clientWidth||el.offsetWidth||500;
  const H  = el.clientHeight||el.offsetHeight||100;
  const dpr= window.devicePixelRatio||1;
  el.width=W*dpr; el.height=H*dpr;
  const ctx=el.getContext('2d');
  ctx.scale(dpr,dpr);
  ctx.clearRect(0,0,W,H);

  const all=[...hist.v1,...hist.v2,...hist.v3].filter(Boolean);
  if(!all.length||hist.v1.length<2) return;
  const mn=Math.min(...all)-0.01;
  const mx=Math.max(...all)+0.01;
  const rng=mx-mn||0.01;
  const n=hist.v1.length;

  /* grid */
  ctx.strokeStyle='#1A221A'; ctx.lineWidth=1;
  for(let r=1;r<5;r++){
    const y=(r/5)*H;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
  }

  const cols=['#39FF14','#0A84FF','#FFB900'];
  ['v1','v2','v3'].forEach((k,ci)=>{
    const arr=hist[k];
    if(arr.length<2) return;
    ctx.beginPath();
    ctx.strokeStyle=cols[ci]; ctx.lineWidth=1.5; ctx.lineJoin='round';
    arr.forEach((v,i)=>{
      const x=(i/(n-1))*W;
      const y=H-((v-mn)/rng)*(H-6)-3;
      i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    });
    ctx.stroke();
  });

  ctx.fillStyle='#556655'; ctx.font='8px JetBrains Mono'; ctx.textAlign='right';
  ctx.fillText(mx.toFixed(3)+'V',W-2,9);
  ctx.fillText(mn.toFixed(3)+'V',W-2,H-2);
}

function $id(id){return document.getElementById(id)}
function bar(id,pct){const e=$id(id);if(e)e.style.width=Math.max(0,Math.min(100,pct))+'%'}
function txt(id,v) {const e=$id(id);if(e)e.textContent=v}

function colorTemp(c){
  if(c>50) return 'var(--red)';
  if(c>40) return 'var(--amber)';
  return 'var(--text)';
}
function colorSOH(h){
  if(h>80) return 'var(--phosphor)';
  if(h>60) return 'var(--amber)';
  return 'var(--red)';
}
function sohLabel(h){
  if(h>80) return 'GOOD';
  if(h>60) return 'AGING';
  return 'REPLACE';
}

function setFault(boxId, stateId, active){
  const box=$id(boxId), st=$id(stateId);
  if(!box||!st) return;
  box.className = 'pfault '+(active?'trip':'ok');
  st.textContent = active ? 'TRIP' : 'OK';
}

function updateAlerts(d){
  const bar=$id('alerts');
  const alerts=[];
  if(d.fuvp) alerts.push({
    title:'Under-Voltage Protection (UVP) Active',
    body: 'A cell has dropped below 3.0 V. Load FET is OFF. Send R to reset after cell recovers.'
  });
  if(d.focp) alerts.push({
    title:'Over-Current Protection (OCP) Active',
    body: 'Current exceeded 3.0 A. Load FET is OFF. Check load and send R to reset.'
  });
  if(d.fotp) alerts.push({
    title:'Over-Temperature Protection (OTP) Active',
    body: 'A cell exceeded 55 °C. Load FET is OFF. Cooling required. Auto-resets below 45 °C.'
  });
  if(d.tm>0) alerts.push({
    title:'SELF-TEST MODE '+(['','UVP','OCP','OTP'][d.tm])+' ACTIVE',
    body: 'Fault injected via command. Send R to clear and resume normal protection.'
  });

  if(alerts.length){
    bar.className='show';
    bar.innerHTML=alerts.map(a=>`
      <div class="alert">
        <div class="alert-icon">⚠</div>
        <div class="alert-body">
          <strong>${a.title}</strong>
          <p>${a.body}</p>
        </div>
      </div>`).join('');
  } else {
    bar.className='';
    bar.innerHTML='';
  }
}

function update(d){
  /* timestamp */
  txt('ts', new Date().toLocaleTimeString());

  /* ── Pack ── */
  txt('pv-soc', d.sp.toFixed(1));
  txt('pv-soh', d.hp.toFixed(1));
  txt('pv-cur', (d.i>=0?'+':'')+d.i.toFixed(2));
  txt('pv-pw',  d.pw.toFixed(2));
  txt('pv-bv',  d.bv.toFixed(2));
  bar('bar-pack-soc', d.sp);

  /* ── Cells ── */
  const cells=[
    {n:1,vk:'v1',tk:'t1',sk:'s1',hk:'h1',rk:'r1'},
    {n:2,vk:'v2',tk:'t2',sk:'s2',hk:'h2',rk:'r2'},
    {n:3,vk:'v3',tk:'t3',sk:'s3',hk:'h3',rk:'r3'},
  ];
  cells.forEach(({n,vk,tk,sk,hk,rk})=>{
    txt('c'+n+'v', d[vk].toFixed(3)+' V');
    txt('c'+n+'t', d[tk].toFixed(1)+' °C');
    txt('c'+n+'s', d[sk].toFixed(1)+'%');
    txt('c'+n+'h', d[hk].toFixed(1)+'%');
    txt('c'+n+'r', (d[rk]*1000).toFixed(1)+' mΩ');
    bar('mb'+n+'s', d[sk]);
    bar('mb'+n+'h', d[hk]);

    /* temperature colour */
    const tel=$id('c'+n+'t');
    if(tel) tel.style.color=colorTemp(d[tk]);

    /* card border on fault */
    const card=$id('cc'+n);
    if(card) card.style.borderColor=(d.fuvp||d.fotp)?'rgba(255,59,48,.4)':'';

    /* thermal readouts on thermal card */
    const thi=$id('th'+n);
    if(thi){ thi.textContent=d[tk].toFixed(1)+' °C'; thi.style.color=colorTemp(d[tk]); }

    /* SOH table */
    const shi=$id('st'+n+'h'), sri=$id('st'+n+'r'), ssi=$id('st'+n+'s');
    if(shi) shi.textContent=d[hk].toFixed(1)+'%';
    if(sri) sri.textContent=(d[rk]*1000).toFixed(1);
    if(ssi){ ssi.textContent=sohLabel(d[hk]); ssi.style.color=colorSOH(d[hk]); }
  });

  /* Pack SOH table */
  txt('stph', d.hp.toFixed(1)+'%');
  const stps=$id('stps');
  if(stps){ stps.textContent=sohLabel(d.hp); stps.style.color=colorSOH(d.hp); }

  /* ── Balancing badges ── */
  ['1','2','3'].forEach(n=>{
    const el=$id('bt'+n);
    if(el) el.className='bal-tag'+(d['b'+n]?' on':'');
  });

  /* ── Cell spread ── */
  const vArr=[d.v1,d.v2,d.v3];
  const vmx=Math.max(...vArr), vmn=Math.min(...vArr);
  txt('vspread', ((vmx-vmn)*1000).toFixed(1)+' mV');
  txt('vmax',    vmx.toFixed(3)+' V');
  txt('vmin',    vmn.toFixed(3)+' V');

  /* ── Protection ── */
  setFault('fi-uvp','fs-uvp',d.fuvp);
  setFault('fi-ocp','fs-ocp',d.focp);
  setFault('fi-otp','fs-otp',d.fotp);

  const led=$id('fet-led'), fval=$id('fet-val');
  if(led){
    led.className='fet-led '+(d.fon?'on':'off');
    fval.className='fet-val '+(d.fon?'on':'off');
    fval.textContent=d.fon?'ON — conducting':'OFF — open';
  }

  /* ── Alerts ── */
  updateAlerts(d);

  /* ── Traces ── */
  pushHist(d);
  drawCellTrace('cv1', hist.v1, '#39FF14');
  drawCellTrace('cv2', hist.v2, '#0A84FF');
  drawCellTrace('cv3', hist.v3, '#FFB900');
  drawPackTrace();
}

/* Redraw on resize */
let resizeT;
window.addEventListener('resize',()=>{ clearTimeout(resizeT); resizeT=setTimeout(()=>{
  drawCellTrace('cv1',hist.v1,'#39FF14');
  drawCellTrace('cv2',hist.v2,'#0A84FF');
  drawCellTrace('cv3',hist.v3,'#FFB900');
  drawPackTrace();
},100); });
</script>
</body>
</html>
)====";

void onWsEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client #%u connected  IP=%s\n",
                      client->id(), client->remoteIP().toString().c_str());
       
        if (lastJson.length() > 2)
            client->text(lastJson);

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len &&
            info->opcode == WS_TEXT && len >= 1)
        {
            char cmd = (char)data[0];
            Serial2.write((uint8_t)cmd);
            Serial.printf("[CMD] Forwarded '%c' to STM32\n", cmd);
        }
    }
}


void processLine(const char *line)
{
    if (line[0] != '{') return;

    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok) {
        Serial.printf("[JSON] Parse fail: %.60s\n", line);
        return;
    }

 
    lastJson = String(line);
    ws.textAll(lastJson);

    Serial.println(line);
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[BMS] ESP32 gateway starting...");

    /*  UART to STM32  */
    Serial2.begin(STM32_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);
    Serial.printf("[BMS] Serial2 RX=GPIO%d  TX=GPIO%d  baud=%d\n",
                  STM32_RX, STM32_TX, STM32_BAUD);

    /*  WiFi  */
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connecting");
    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries++ < 40) {
        delay(500); Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected!  IP: %s\n",
                      WiFi.localIP().toString().c_str());
        Serial.printf("[HTTP] Dashboard: http://%s/\n",
                      WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] FAILED — check credentials.");
    }

    /*  WebSocket  */
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    /*  HTTP: dashboard page  */
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
        req->send_P(200, "text/html", PAGE);
    });

    /*  HTTP: REST JSON snapshot */
    server.on("/api", HTTP_GET, [](AsyncWebServerRequest *req){
        req->send(200, "application/json", lastJson);
    });

    /* HTTP: 404  */
    server.onNotFound([](AsyncWebServerRequest *req){
        req->send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.println("[HTTP] Server started");
}

void loop()
{
    /* Read from STM32 */
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n') {
            if (line_pos > 0) {
                line_buf[line_pos] = '\0';
                processLine(line_buf);
                line_pos = 0;
            }
        } else if (c != '\r') {
            if (line_pos < LINE_BUF - 1)
                line_buf[line_pos++] = c;
            else
                line_pos = 0;   
        }
    }

  
    ws.cleanupClients();
    yield();
}
