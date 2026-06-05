
/*
  ESP32 Single Traffic Light Wi-Fi Node
  - One ESP32 controls one 3-color lamp (R/Y/G/GND)
  - Commands come from PyQt central controller over TCP
  - This firmware is identical for all 4 ESP32 boards.
  - Change DEVICE_ID, WIFI_SSID, WIFI_PASSWORD before upload.

  Supported commands (one line each, '\n' terminated):
    PING
    STATUS
    SET R
    SET Y
    SET G
    SET OFF
    BLINK R 500
    BLINK Y 500
    BLINK OFF

  Replies:
    PONG
    STATUS,ID=1,LAMP=R,BLINK=OFF,BLINK_LAMP=OFF
    OK,SET,R
    OK,BLINK,START
    ERR,...

  Hardware:
    R -> GPIO 25
    Y -> GPIO 26
    G -> GPIO 27
    GND -> ESP32 GND

  NOTE:
    If your lamp current is large, do NOT drive it directly from ESP32.
    Use transistor / MOSFET / relay driver as needed.
*/

#include <WiFi.h>

#define PIN_R 27
#define PIN_Y 26
#define PIN_G 25

// ============================
// USER SETUP
// ============================
#define DEVICE_ID 1   // Change to 1, 2, 3, 4 on each ESP32

const char* WIFI_SSID     = "addinedu_201class_4-2.4G";
const char* WIFI_PASSWORD = "201class4!";
const uint16_t SERVER_PORT = 5000;
// ============================

enum LampState {
  LAMP_OFF,
  LAMP_R,
  LAMP_Y,
  LAMP_G
};

WiFiServer server(SERVER_PORT);
WiFiClient client;

LampState currentLamp = LAMP_R;

// blink
bool blinkEnabled = false;
LampState blinkLamp = LAMP_OFF;
bool blinkOn = false;
unsigned long blinkInterval = 500;
unsigned long lastBlinkMillis = 0;

String rxBuffer = "";

void applyLamp(LampState state) {
  currentLamp = state;
  digitalWrite(PIN_R, state == LAMP_R ? HIGH : LOW);
  digitalWrite(PIN_Y, state == LAMP_Y ? HIGH : LOW);
  digitalWrite(PIN_G, state == LAMP_G ? HIGH : LOW);
}

const char* lampToString(LampState s) {
  switch (s) {
    case LAMP_R: return "R";
    case LAMP_Y: return "Y";
    case LAMP_G: return "G";
    default: return "OFF";
  }
}

void sendLine(const String& msg) {
  if (client && client.connected()) {
    client.println(msg);
  }
  Serial.println(msg);
}

void sendStatus() {
  String msg = "STATUS,ID=" + String(DEVICE_ID)
             + ",LAMP=" + String(lampToString(currentLamp))
             + ",BLINK=" + String(blinkEnabled ? "ON" : "OFF")
             + ",BLINK_LAMP=" + String(lampToString(blinkLamp));
  sendLine(msg);
}

void resetEffects() {
  blinkEnabled = false;
  blinkLamp = LAMP_OFF;
  blinkOn = false;
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print("[RX] ");
  Serial.println(cmd);

  if (cmd.equalsIgnoreCase("PING")) {
    sendLine("PONG");
    return;
  }

  if (cmd.equalsIgnoreCase("STATUS")) {
    sendStatus();
    return;
  }

  if (cmd.startsWith("SET ")) {
    String target = cmd.substring(4);
    target.trim();
    resetEffects();

    if (target.equalsIgnoreCase("R")) {
      applyLamp(LAMP_R);
      sendLine("OK,SET,R");
    } else if (target.equalsIgnoreCase("Y")) {
      applyLamp(LAMP_Y);
      sendLine("OK,SET,Y");
    } else if (target.equalsIgnoreCase("G")) {
      applyLamp(LAMP_G);
      sendLine("OK,SET,G");
    } else if (target.equalsIgnoreCase("OFF")) {
      applyLamp(LAMP_OFF);
      sendLine("OK,SET,OFF");
    } else {
      sendLine("ERR,UNKNOWN_SET");
    }
    return;
  }

  if (cmd.startsWith("BLINK ")) {
    String rest = cmd.substring(6);
    rest.trim();

    if (rest.equalsIgnoreCase("OFF")) {
      resetEffects();
      applyLamp(LAMP_OFF);
      sendLine("OK,BLINK,OFF");
      return;
    }

    int sp = rest.indexOf(' ');
    if (sp < 0) {
      sendLine("ERR,BAD_BLINK");
      return;
    }

    String color = rest.substring(0, sp);
    String intervalStr = rest.substring(sp + 1);
    intervalStr.trim();
    unsigned long intervalVal = intervalStr.toInt();

    if (intervalVal < 100) {
      sendLine("ERR,INVALID_BLINK_INTERVAL");
      return;
    }

    if (color.equalsIgnoreCase("R")) {
      blinkLamp = LAMP_R;
    } else if (color.equalsIgnoreCase("Y")) {
      blinkLamp = LAMP_Y;
    } else {
      sendLine("ERR,BLINK_ONLY_R_OR_Y");
      return;
    }

    blinkEnabled = true;
    blinkInterval = intervalVal;
    lastBlinkMillis = millis();
    blinkOn = true;
    applyLamp(blinkLamp);
    sendLine("OK,BLINK,START");
    return;
  }

  sendLine("ERR,UNKNOWN_COMMAND");
}

void acceptClientIfNeeded() {
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client.stop();
      client = newClient;
      client.println("HELLO,TRAFFIC_LIGHT_NODE,ID=" + String(DEVICE_ID));
      sendStatus();
      Serial.println("[INFO] Controller connected");
    }
  }
}

void readClientLines() {
  if (!(client && client.connected())) return;

  while (client.available()) {
    char c = (char)client.read();
    if (c == '\n') {
      handleCommand(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
    }
  }
}

void processBlink() {
  if (!blinkEnabled) return;

  unsigned long now = millis();
  if (now - lastBlinkMillis >= blinkInterval) {
    lastBlinkMillis = now;
    blinkOn = !blinkOn;
    if (blinkOn) applyLamp(blinkLamp);
    else applyLamp(LAMP_OFF);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_Y, OUTPUT);
  pinMode(PIN_G, OUTPUT);

  applyLamp(LAMP_R);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[INFO] Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("[INFO] Wi-Fi connected");
  Serial.print("[INFO] DEVICE_ID=");
  Serial.println(DEVICE_ID);
  Serial.print("[INFO] IP=");
  Serial.println(WiFi.localIP());
  Serial.print("[INFO] PORT=");
  Serial.println(SERVER_PORT);

  server.begin();
  Serial.println("[INFO] TCP server started");
}

void loop() {
  acceptClientIfNeeded();
  readClientLines();
  processBlink();
}
