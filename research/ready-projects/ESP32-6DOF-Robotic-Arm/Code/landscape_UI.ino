#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "ArmController";
const char* AP_PASS = "12345678";

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define NUM_SERVOS 6
#define SERVO_MIN  200
#define SERVO_MAX  550

int CHANNELS[NUM_SERVOS]  = {0, 1, 2, 3, 8, 9};
const char* labels[]      = {"Base", "Lower", "Mid", "Upper", "Wrist", "Gripper"};
const char* types[]       = {"MG996R", "MG996R", "MG996R", "MG90S", "MG90S", "MG90S"};
int ANGLE_MIN[NUM_SERVOS] = {0,   0,   0,   0,   0,   0};
int ANGLE_MAX[NUM_SERVOS] = {180, 180, 180, 180, 180, 120};
int angles[NUM_SERVOS]    = {90,  90,  90,  90,  90,  90};

WebServer server(80);

int degreesToPulse(int deg) {
  return map(constrain(deg, 0, 180), 0, 180, SERVO_MIN, SERVO_MAX);
}

void moveServo(int idx, int deg) {
  angles[idx] = constrain(deg, ANGLE_MIN[idx], ANGLE_MAX[idx]);
  pca.setPWM(CHANNELS[idx], 0, degreesToPulse(angles[idx]));
}

void centerAll() {
  for (int i = 0; i < NUM_SERVOS; i++) moveServo(i, 90);
}

void maxAll() {
  for (int i = 0; i < NUM_SERVOS; i++) moveServo(i, 180);
}

const char HTML_PAGE[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Arm Controller</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    font-family: 'Segoe UI', Arial, sans-serif;
    background: #f0f4f8;
    color: #1a202c;
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }

  /* TOP BAR */
  .topbar {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px 24px;
    background: #ffffff;
    border-bottom: 2px solid #e2e8f0;
    box-shadow: 0 1px 4px rgba(0,0,0,0.08);
  }
  .topbar h1 {
    color: #16a34a;
    font-size: 16px;
    letter-spacing: 2px;
    font-weight: 700;
  }
  .topbar .info {
    font-size: 10px;
    color: #94a3b8;
    margin-top: 2px;
  }
  .btn-group { display: flex; gap: 8px; }
  .topbtn {
    padding: 7px 16px;
    border-radius: 6px;
    font-size: 12px;
    font-weight: 600;
    cursor: pointer;
    border: none;
    letter-spacing: 0.5px;
  }
  .btn-center {
    background: #dcfce7;
    color: #16a34a;
    border: 1px solid #86efac;
  }
  .btn-center:hover { background: #bbf7d0; }
  .btn-max {
    background: #fef3c7;
    color: #d97706;
    border: 1px solid #fcd34d;
  }
  .btn-max:hover { background: #fde68a; }

  /* MAIN GRID */
  .main {
    flex: 1;
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 0;
    overflow: hidden;
  }

  .panel {
    padding: 12px 16px;
    border-right: 2px solid #e2e8f0;
    display: flex;
    flex-direction: column;
    gap: 8px;
    overflow-y: auto;
    background: #f8fafc;
  }
  .panel:last-child { border-right: none; background: #f0f4f8; }

  .panel-title {
    font-size: 11px;
    font-weight: 700;
    color: #d97706;
    letter-spacing: 1.5px;
    padding-bottom: 6px;
    border-bottom: 2px solid #fde68a;
    margin-bottom: 2px;
    text-transform: uppercase;
  }

  /* SERVO CARD */
  .card {
    background: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 10px;
    padding: 12px 14px;
    box-shadow: 0 1px 3px rgba(0,0,0,0.06);
  }
  .card-top {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 10px;
  }
  .sname {
    font-size: 13px;
    font-weight: 700;
    color: #16a34a;
    letter-spacing: 0.5px;
  }
  .stype { font-size: 10px; color: #94a3b8; margin-top: 1px; }
  .sch   { font-size: 10px; color: #3b82f6; margin-top: 1px; font-weight: 600; }
  .live-badge {
    background: #fffbeb;
    border: 1px solid #fcd34d;
    border-radius: 6px;
    padding: 3px 10px;
    font-size: 14px;
    font-weight: 700;
    color: #d97706;
    min-width: 52px;
    text-align: center;
  }

  /* SLIDER */
  .slider-row {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 8px;
  }
  .slim {
    font-size: 10px;
    color: #94a3b8;
    width: 24px;
    text-align: center;
    font-weight: 600;
  }
  input[type=range] {
    flex: 1;
    accent-color: #16a34a;
    cursor: pointer;
    height: 5px;
  }

  /* NUMBER INPUT ROW */
  .input-row {
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .input-label {
    font-size: 10px;
    color: #64748b;
    font-weight: 600;
    white-space: nowrap;
  }
  .deg-input {
    width: 64px;
    background: #f8fafc;
    border: 1px solid #cbd5e1;
    border-radius: 5px;
    color: #d97706;
    font-size: 13px;
    font-weight: 700;
    padding: 4px 6px;
    text-align: center;
  }
  .deg-input:focus {
    outline: none;
    border-color: #16a34a;
    background: #f0fdf4;
  }
  .set-btn {
    padding: 4px 12px;
    background: #dcfce7;
    border: 1px solid #86efac;
    border-radius: 5px;
    color: #16a34a;
    font-size: 11px;
    font-weight: 700;
    cursor: pointer;
  }
  .set-btn:hover { background: #bbf7d0; }
  .unit { font-size: 10px; color: #94a3b8; }
</style>

<script>
function setServo(idx, angle) {
  angle = Math.max(0, Math.min(180, parseInt(angle) || 0));
  fetch('/set?idx=' + idx + '&angle=' + angle).then(r => r.text()).then(v => {
    v = parseInt(v);
    document.getElementById('sl' + idx).value = v;
    document.getElementById('ni' + idx).value = v;
    document.getElementById('lb' + idx).innerText = v + 'deg';
  });
}

function onSlider(idx) {
  let v = document.getElementById('sl' + idx).value;
  document.getElementById('ni' + idx).value = v;
  document.getElementById('lb' + idx).innerText = v + 'deg';
  fetch('/set?idx=' + idx + '&angle=' + v);
}

function onInput(idx) {
  let v = document.getElementById('ni' + idx).value;
  document.getElementById('lb' + idx).innerText = v + 'deg';
}

function centerAll() {
  fetch('/center').then(() => {
    for (let i = 0; i < 6; i++) {
      document.getElementById('sl' + i).value = 90;
      document.getElementById('ni' + i).value = 90;
      document.getElementById('lb' + i).innerText = '90deg';
    }
  });
}

function maxAll() {
  fetch('/max').then(r => r.text()).then(data => {
    let vals = JSON.parse(data);
    for (let i = 0; i < 6; i++) {
      document.getElementById('sl' + i).value = vals[i];
      document.getElementById('ni' + i).value = vals[i];
      document.getElementById('lb' + i).innerText = vals[i] + 'deg';
    }
  });
}
</script>
</head><body>

<div class='topbar'>
  <div>
    <h1>ARM CONTROLLER</h1>
    <div class='info'>ESP32 + PCA9685 &nbsp;·&nbsp; 192.168.4.1 &nbsp;·&nbsp; 6 Servos</div>
  </div>
  <div class='btn-group'>
    <button class='topbtn btn-center' onclick='centerAll()'>⟳ CENTER ALL (90°)</button>
    <button class='topbtn btn-max'    onclick='maxAll()'>↑ MAX ALL (180°)</button>
  </div>
</div>

<div class='main'>
  <div class='panel'>
    <div class='panel-title'>Lower Arm &nbsp;·&nbsp; Base / Lower / Mid</div>
    LOWER_CARDS
  </div>
  <div class='panel'>
    <div class='panel-title'>Upper Arm &nbsp;·&nbsp; Upper / Wrist / Gripper</div>
    UPPER_CARDS
  </div>
</div>

</body></html>)rawhtml";

String buildCard(int i) {
  String c = "<div class='card'>";
  c += "<div class='card-top'>";
  c += "<div>";
  c += "<div class='sname'>"; c += labels[i]; c += "</div>";
  c += "<div class='stype'>"; c += types[i]; c += "</div>";
  c += "<div class='sch'>Channel "; c += CHANNELS[i]; c += "</div>";
  c += "</div>";
  c += "<div class='live-badge' id='lb"; c += i; c += "'>"; c += angles[i]; c += "deg</div>";
  c += "</div>";

  // Slider
  c += "<div class='slider-row'>";
  c += "<span class='slim'>"; c += ANGLE_MIN[i]; c += "</span>";
  c += "<input type='range' id='sl"; c += i;
  c += "' min='"; c += ANGLE_MIN[i];
  c += "' max='"; c += ANGLE_MAX[i];
  c += "' value='"; c += angles[i];
  c += "' oninput='onSlider("; c += i; c += ")'>";
  c += "<span class='slim'>"; c += ANGLE_MAX[i]; c += "</span>";
  c += "</div>";

  // Number input + SET
  c += "<div class='input-row'>";
  c += "<span class='input-label'>Type degrees:</span>";
  c += "<input class='deg-input' type='number' id='ni"; c += i;
  c += "' min='"; c += ANGLE_MIN[i];
  c += "' max='"; c += ANGLE_MAX[i];
  c += "' value='"; c += angles[i];
  c += "' oninput='onInput("; c += i; c += ")'>";
  c += "<button class='set-btn' onclick='setServo("; c += i;
  c += ", document.getElementById(\"ni"; c += i; c += "\").value)'>SET</button>";
  c += "<span class='unit'>deg</span>";
  c += "</div>";
  c += "</div>";
  return c;
}

void handleRoot() {
  String lower_cards = buildCard(0) + buildCard(1) + buildCard(2);
  String upper_cards = buildCard(3) + buildCard(4) + buildCard(5);
  String page = FPSTR(HTML_PAGE);
  page.replace("LOWER_CARDS", lower_cards);
  page.replace("UPPER_CARDS", upper_cards);
  server.send(200, "text/html", page);
}

void handleSet() {
  if (server.hasArg("idx") && server.hasArg("angle")) {
    int idx   = server.arg("idx").toInt();
    int angle = server.arg("angle").toInt();
    if (idx >= 0 && idx < NUM_SERVOS) {
      moveServo(idx, angle);
      Serial.print(labels[idx]); Serial.print(": "); Serial.println(angles[idx]);
      server.send(200, "text/plain", String(angles[idx]));
      return;
    }
  }
  server.send(400, "text/plain", "err");
}

void handleCenter() {
  centerAll();
  Serial.println(">> All -> 90");
  server.send(200, "text/plain", "ok");
}

void handleMax() {
  maxAll();
  Serial.println(">> All -> 180");
  // Return JSON array of actual angles (gripper capped at 120)
  String json = "[";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += String(angles[i]);
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n>> Starting...");

  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);
  delay(300);
  Serial.println(">> PCA9685 OK");

  Serial.println(">> Centering servos...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, 90);
    delay(300);
    Serial.print("   Ch"); Serial.print(CHANNELS[i]);
    Serial.print(" "); Serial.print(labels[i]); Serial.println(" -> 90");
  }
  Serial.println(">> Servos OK");

  btStop();
  delay(100);
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_AP);
  delay(500);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  delay(1000);

  if (ok) {
    Serial.println(">> Hotspot OK!");
    Serial.print(">> IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("!! Hotspot FAILED - restarting...");
    delay(2000);
    ESP.restart();
  }

  server.on("/",       handleRoot);
  server.on("/set",    handleSet);
  server.on("/center", handleCenter);
  server.on("/max",    handleMax);
  server.begin();

  Serial.println("==============================");
  Serial.println("WiFi:  ArmController");
  Serial.println("Pass:  12345678");
  Serial.println("URL:   http://192.168.4.1");
  Serial.println("==============================");
  Serial.println(">> Ready!");
}

void loop() {
  server.handleClient();
}