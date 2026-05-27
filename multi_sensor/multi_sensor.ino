// ============================================================
// multi_sensor.ino  —  ESP32 Sensor & Actuator Interface
// ============================================================
// Protocol:
//   Arduino → PC : "T:<pot_C>,RT:<room_C>\n"  (every ~1 s)
//   PC → Arduino : "S<milliseconds>\n"  e.g. "S3500\n"
//   Arduino → PC : "ACK:SALT_DONE\n"   (after motor stops)
//
// EC is measured manually with a commercial sensor and entered
// on the PC side. Arduino only handles temperature + motor.
// ============================================================

#include <Adafruit_MAX31865.h>
#include <DHT.h>

// --- MAX31865 (PT100) ---
Adafruit_MAX31865 thermo = Adafruit_MAX31865(5);
#define RREF      430.0
#define RNOMINAL  100.0

// --- DHT22 (room temperature) ---
#define DHT_PIN  14
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// --- Salt mill relay ---
const int RELAY_PIN    = 27;
const int MOTOR_MAX_MS = 25000;  // safety cap: 25 s

// --- Timing ---
unsigned long lastSendMs = 0;
const unsigned long SEND_INTERVAL_MS = 1000;

// ── センサーデータをCSV形式で送信 ──
void sendSensorData() {
  float potTemp  = thermo.temperature(RNOMINAL, RREF);
  float roomTemp = dht.readTemperature();

  Serial.print("T:");
  Serial.print(potTemp, 2);
  Serial.print(",RT:");
  if (isnan(roomTemp)) {
    Serial.print("nan");
  } else {
    Serial.print(roomTemp, 2);
  }
  Serial.println();

  // PT100エラーチェック
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("ERR:PT100_FAULT_0x");
    Serial.println(fault, HEX);
    thermo.clearFault();
  }
}

// ── ソルトミルをms[ミリ秒]だけ動かす ──
void runSaltMill(int duration_ms) {
  digitalWrite(RELAY_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("ACK:SALT_DONE");
}

// ── シリアルコマンドの処理 ──
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("S") || cmd.startsWith("s")) {
    int duration_ms = cmd.substring(1).toInt();
    if (duration_ms > 0 && duration_ms <= MOTOR_MAX_MS) {
      runSaltMill(duration_ms);
    } else {
      Serial.print("ERR:INVALID_DURATION:");
      Serial.println(duration_ms);
    }
  } else {
    Serial.print("ERR:UNKNOWN_CMD:");
    Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  thermo.begin(MAX31865_3WIRE);
  dht.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("READY");
}

void loop() {
  // コマンド受信チェック
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // 1秒ごとにセンサーデータ送信
  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    sendSensorData();
  }
}
