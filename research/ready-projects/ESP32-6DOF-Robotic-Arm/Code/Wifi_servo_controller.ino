#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "RoboArm";
const char* AP_PASS = "robo1234";

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define NUM_SERVOS 6
#define SERVO_MIN  200
#define SERVO_MAX  550

int CHANNELS[NUM_SERVOS]  = {0, 1, 2, 3, 8, 9};
const char* labels[]      = {"Base", "Lower", "Mid", "Upper", "Wrist", "Gripper"};
const char* types[]       = {"MG996R", "MG996R", "MG996R", "MG90S", "MG90S", "MG90S"};
int ANGLE_MIN[NUM_SERVOS] = {0,   0,   0,   0,   0,   20};
int ANGLE_MAX[NUM_SERVOS] = {180, 180, 180, 180, 180, 120};
int angles[NUM_SERVOS]    = {90,  90,  90,  90,  90,  90};

WebServer server(80);

// ── HTML stored in PROGMEM (flash) not RAM ───────────────────
const char HTML_HEAD[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>
<title>Arm Controller</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{font-family:monospace;background:#0a0a0f;color:#e2e8f0;padding:12px;max-width:480px;margin:0 auto}
h1{color:#22c55e;font-size:15px;letter-spacing:3px;margin-bottom:2px}
.sub{font-size:9px;color:#374151;margin-bottom:14px}
.card{background:#0f0f1a;border:1px solid #1e293b;border-radius:10px;padding:12px;margin-bottom:8px}
.card-top{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.sname{font-size:11px;color:#22c55e;letter-spacing:1px}
.stype{font-size:9px;color:#374151;margin-top:1px}
.badge{background:#0a0a0f;border:1px solid #f59e0b55;border-radius:6px;padding:4px 10px;font-size:15px;font-weight:bold;color:#f59e0b;min-width:54px;text-align:center}
.jrow{display:flex;align-items:center;gap:6px;margin-top:4px}
.jbtn{width:44px;height:44px;border-radius:8px;border:1px solid #22c55e44;background:#0a0a0f;color:#22c55e;font-size:20px;cursor:pointer;user-select:none;touch-action:none;display:flex;align-items:center;justify-content:center}
.jbtn:active{background:#22c55e33}
.jbtn.fast{border-color:#f59e0b44;color:#f59e0b;font-size:11px}
.jbtn.fast:active{background:#f59e0b22}
.jbar{flex:1;height:8px;background:#1e293b;border-radius:4px;overflow:hidden}
.jfill{height:100%;background:#22c55e;border-radius:4px;transition:width 0.1s}
.summary{background:#0f0f1a;border:1px solid #f59e0b22;border-radius:10px;padding:12px;margin-top:4px}
.stitle{font-size:9px;color:#f59e0b;letter-spacing:2px;margin-bottom:8px}
.sgrid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px}
.sitem{background:#0a0a0f;border-radius:5px;padding:6px 8px;text-align:center}
.slabel{font-size:8px;color:#4a5568;margin-bottom:2px}
.sval{font-size:13px;font-weight:bold;color:#f59e0b}
.cbtn{width:100%;padding:13px;margin-top:8px;background:#22c55e12;border:1px solid #22c55e33;border-radius:8px;color:#22c55e;font-size:11px;font-family:monospace;cursor:pointer;letter-spacing:2px}
.cbtn:active{background:#22c55e25}
</style>
<script>
let ht=null;
function mv(i,d){
  fetch('/move?idx='+i+'&delta='+d).then(r=>r.text()).then(v=>{
    v=parseInt(v);
    document.getElementById('b'+i).innerText=v+'d';
    document.getElementById('s'+i).innerText=v+'d';
    document.getElementById('f'+i).style.width=(v/180*100)+'%';
  });
}
function sh(i,d){mv(i,d);ht=setInterval(()=>mv(i,d),150);}
function sp(){clearInterval(ht);}
function ca(){
  fetch('/center').then(()=>{
    for(let i=0;i<6;i++){
      document.getElementById('b'+i).innerText='90d';
      document.getElementById('s'+i).innerText='90d';
      document.getElementById('f'+i).style.width='50%';
    }
  });
}
</script>
</head><body>
<h1>ARM CONTROLLER</h1>
<div class='sub'>6 SERVOS · ESP32 + PCA9685 · 192.168.4.1</div>)rawhtml";

const char HTML_TAIL[] PROGMEM = R"rawhtml(
<div class='summary'>
<div class='stitle'>CURRENT ANGLES</div>
<div class='sgrid' id='grid'></div>
</div>
<button class='cbtn' onclick='ca()'>CENTER ALL TO 90</button>
</body></html>)rawhtml";

// ── SERVO CONTROL ───────────────────────────────────────────
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

// ── HANDLERS ────────────────────────────────────────────────
void handleRoot() {
  String page = "";
  // Head from PROGMEM
  page += FPSTR(HTML_HEAD);

  // Generate cards dynamically (small per-card strings)
  for (int i = 0; i < NUM_SERVOS; i++) {
    int pct = (int)(angles[i] / 180.0 * 100);
    page += "<div class='card'><div class='card-top'>";
    page += "<div><div class='sname'>CH";
    page += CHANNELS[i];
    page += " - ";
    page += labels[i];
    page += "</div><div class='stype'>";
    page += types[i];
    page += "</div></div><div class='badge' id='b";
    page += i;
    page += "'>";
    page += angles[i];
    page += "d</div></div><div class='jrow'>";
    page += "<button class='jbtn fast' onpointerdown='sh(";
    page += i;
    page += ",-15)' onpointerup='sp()' onpointerleave='sp()'>-15</button>";
    page += "<button class='jbtn' onpointerdown='sh(";
    page += i;
    page += ",-5)' onpointerup='sp()' onpointerleave='sp()'>&#9664;</button>";
    page += "<div class='jbar'><div class='jfill' id='f";
    page += i;
    page += "' style='width:";
    page += pct;
    page += "%'></div></div>";
    page += "<button class='jbtn' onpointerdown='sh(";
    page += i;
    page += ",5)' onpointerup='sp()' onpointerleave='sp()'>&#9654;</button>";
    page += "<button class='jbtn fast' onpointerdown='sh(";
    page += i;
    page += ",15)' onpointerup='sp()' onpointerleave='sp()'>+15</button>";
    page += "</div></div>";
  }

  // Summary grid
  page += "<div class='summary'><div class='stitle'>CURRENT ANGLES</div><div class='sgrid'>";
  for (int i = 0; i < NUM_SERVOS; i++) {
    page += "<div class='sitem'><div class='slabel'>";
    page += labels[i];
    page += "</div><div class='sval' id='s";
    page += i;
    page += "'>";
    page += angles[i];
    page += "d</div></div>";
  }
  page += "</div></div>";
  page += "<button class='cbtn' onclick='ca()'>CENTER ALL TO 90</button>";
  page += "</body></html>";

  server.send(200, "text/html", page);
}

void handleMove() {
  if (server.hasArg("idx") && server.hasArg("delta")) {
    int idx   = server.arg("idx").toInt();
    int delta = server.arg("delta").toInt();
    if (idx >= 0 && idx < NUM_SERVOS) {
      moveServo(idx, angles[idx] + delta);
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

// ── SETUP ───────────────────────────────────────────────────
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

  // Start hotspot
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
  server.on("/move",   handleMove);
  server.on("/center", handleCenter);
  server.begin();

  Serial.println("==============================");
  Serial.println("WiFi: RoboArm");
  Serial.println("Pass:  robo1234");
  Serial.println("URL:   http://192.168.4.1");
  Serial.println("==============================");
  Serial.println(">> Ready!");
}

void loop() {
  server.handleClient();
}