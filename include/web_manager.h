#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <LittleFS.h>
#include <math.h>
#include "device_config.h"
#include "data_logger.h"

class WebManager {
public:
  WebManager(DataLogger &logger) : logger_(logger) {}
  void begin() {
    WiFi.softAPConfig(apAddress_, apAddress_, IPAddress(255, 255, 255, 0));
    if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
      Serial.println("[WEB] AP start failed");
      return;
    }
    Serial.print("[WEB] AP IP: "); Serial.println(WiFi.softAPIP());
    server_.on("/", HTTP_GET, [this]() { server_.send_P(200, "text/html; charset=utf-8", PAGE); });
    server_.on("/api/status", HTTP_GET, [this]() { status(); });
    server_.on("/api/log/current", HTTP_GET, [this]() { download("/temperature.csv"); });
    server_.on("/api/log/previous", HTTP_GET, [this]() { download("/temperature_previous.csv"); });
    server_.onNotFound([this]() { server_.send(404, "text/plain", "Not found"); });
    // HTTP Basic auth is not encrypted. Use only on a secured local AP.
    updater_.setup(&server_, "/update", ADMIN_USER, ADMIN_PASSWORD);
    server_.begin();
    started_ = true;
  }
  void poll() { if (started_) server_.handleClient(); }
  void setTelemetry(double temp, double target, double oxygen, double pwm, const char* oxygenStatus) {
    temp_ = temp; target_ = target; oxygen_ = oxygen; pwm_ = pwm;
    oxygenStatus_ = oxygenStatus;
  }
private:
  static constexpr const char* PAGE = R"HTML(<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Photoreactor</title><style>
  :root{color-scheme:dark;font-family:system-ui,Arial,sans-serif}body{margin:0;background:#0c1220;color:#eef4ff}.wrap{max-width:900px;margin:auto;padding:24px}header{display:flex;justify-content:space-between;align-items:center;gap:12px}.muted{color:#99a9c5}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-top:24px}.card{background:#18243a;border:1px solid #2e4059;border-radius:14px;padding:20px}.value{font-size:36px;font-weight:700;margin:7px 0}a{color:#74c9ff}button{background:#326dba;border:0;border-radius:8px;color:white;padding:10px 16px;cursor:pointer}canvas{width:100%;height:220px}small{color:#99a9c5}@media(max-width:600px){.value{font-size:27px}.wrap{padding:15px}}
  </style></head><body><div class="wrap"><header><div><h1>Photoreactor</h1><div class="muted" id="identity">Connecting...</div></div><span id="connection">●</span></header>
  <div class="grid"><div class="card"><small>현재 온도</small><div class="value" id="temp">--</div></div><div class="card"><small>목표 온도</small><div class="value" id="target">--</div></div><div class="card"><small>O₂ 농도</small><div class="value" id="oxygen">--</div><small id="o2status"></small></div><div class="card"><small>펠티어 PWM</small><div class="value" id="pwm">--</div></div></div>
  <div class="card" style="margin-top:16px"><h2>실시간 온도 (이 브라우저 접속 이후)</h2><canvas id="graph" width="860" height="240"></canvas><small>그래프는 브라우저 메모리만 사용합니다. 보존된 기록은 CSV를 내려받으세요.</small></div>
  <div class="card" style="margin-top:16px"><h2>기록 및 관리</h2><p><a href="/api/log/current">현재 CSV 다운로드</a> · <a href="/api/log/previous">이전 CSV 다운로드</a></p><p><a href="/update">관리자 펌웨어 업데이트</a></p><small>CSV 시간은 UTC가 아닌 Pico 부팅 이후 밀리초입니다. OTA 중에는 온도 제어가 중단될 수 있으므로 실험 종료 후 업데이트하세요.</small></div></div>
  <script>let samples=[];const el=id=>document.getElementById(id);function fmt(v,s){return v===null?'ERROR':Number(v).toFixed(2)+s}function draw(){let c=el('graph'),g=c.getContext('2d'),w=c.width,h=c.height;g.clearRect(0,0,w,h);g.strokeStyle='#4b5c73';for(let i=0;i<5;i++){let y=20+i*(h-40)/4;g.beginPath();g.moveTo(28,y);g.lineTo(w-15,y);g.stroke()}if(samples.length<2)return;let values=samples.flatMap(v=>[v.t,v.target]).filter(Number.isFinite);let lo=Math.min(...values)-2,hi=Math.max(...values)+2;if(hi-lo<1)hi=lo+1;for(let k=0;k<2;k++){g.strokeStyle=k?'#e4af66':'#48b8ed';g.lineWidth=2;g.beginPath();samples.forEach((s,i)=>{let v=k?s.target:s.t;if(v===null)return;let x=28+i*(w-43)/(samples.length-1),y=20+(hi-v)*(h-40)/(hi-lo);if(i===0)g.moveTo(x,y);else g.lineTo(x,y)});g.stroke()}}async function tick(){try{let r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error(r.status);let x=await r.json();el('identity').textContent=x.device+' · FW '+x.firmware+' · '+Math.floor(x.uptime_ms/1000)+'s uptime';el('temp').textContent=fmt(x.temperature_c,'°C');el('target').textContent=fmt(x.target_c,'°C');el('oxygen').textContent=fmt(x.oxygen_pct,'%');el('o2status').textContent=x.oxygen_status;el('pwm').textContent=fmt(x.pwm_pct,'%');el('connection').textContent='● Online';samples.push({t:x.temperature_c,target:x.target_c});if(samples.length>3600)samples.shift();draw()}catch(e){el('connection').textContent='● Offline'}}tick();setInterval(tick,1000);</script></body></html>)HTML";
  static String number(double value, uint8_t decimals) {
    return isfinite(value) ? String(value, decimals) : String("null");
  }
  void status() {
    String json = "{\"device\":\"" DEVICE_NAME "\",\"firmware\":\"" FW_VERSION "\",\"uptime_ms\":";
    json += String(millis());
    json += ",\"temperature_c\":" + number(temp_, 2);
    json += ",\"target_c\":" + number(target_, 2);
    json += ",\"oxygen_pct\":" + number(oxygen_, 2);
    json += ",\"pwm_pct\":" + number(pwm_, 1);
    json += ",\"oxygen_status\":\"" + String(oxygenStatus_) + "\"}";
    server_.sendHeader("Cache-Control", "no-store");
    server_.send(200, "application/json", json);
  }
  void download(const char* path) {
    if (!logger_.ready) { server_.send(503, "text/plain", "Storage unavailable"); return; }
    File f = LittleFS.open(path, "r");
    if (!f) { server_.send(404, "text/plain", "No log yet"); return; }
    // WebServer::streamFile sends a known content length.
    String filename = String(path).substring(1);
    server_.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    server_.streamFile(f, "text/csv");
    f.close();
  }
  const IPAddress apAddress_{192, 168, 42, 1};
  DataLogger &logger_;
  WebServer server_{apAddress_, 80};
  HTTPUpdateServer updater_;
  bool started_ = false;
  double temp_ = NAN, target_ = 0, oxygen_ = NAN, pwm_ = 0;
  const char* oxygenStatus_ = "starting";
};
