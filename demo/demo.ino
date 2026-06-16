const int RELAY_PIN    = 27;
const int MOTOR_MAX_MS = 25000;  // 安全上限 25s

void runSaltMill(int duration_ms) {
  digitalWrite(RELAY_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("ACK:SALT_DONE");
}

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
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}