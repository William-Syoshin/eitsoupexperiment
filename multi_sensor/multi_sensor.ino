#include <Adafruit_MAX31865.h>

// --- MAX31865 (PT100) の設定 ---
Adafruit_MAX31865 thermo = Adafruit_MAX31865(5);
#define RREF      430.0
#define RNOMINAL  100.0

// --- 伝導率(TDS/EC)センサーの設定 ---
const int EC_PIN = 32;

// --- ソルトミル（relay-T73モジュール）の設定 ---
const int RELAY_PIN = 27; // リレーのIN（S / SIG）端子に繋ぐピン

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 - Robotic Chef Sensor & Actuator Test!");

  // PT100を3線式で初期化
  thermo.begin(MAX31865_3WIRE);

  // リレー制御ピンの初期化と安全対策
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // 起動時は必ずOFF（モーター停止）にしておく
}

void loop() {
  // 1. PT100から温度を取得
  float temperature = thermo.temperature(RNOMINAL, RREF);
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" C");

  // 2. 伝導率センサーからアナログ値と電圧を取得
  int ecAnalogValue = analogRead(EC_PIN);
  float ecVoltage = ecAnalogValue * (3.3 / 4095.0);

  Serial.print("Conductivity Analog = ");
  Serial.print(ecAnalogValue);
  Serial.print(" (Voltage: ");
  Serial.print(ecVoltage);
  Serial.println(" V)");

  // 3. エラーチェック（PT100の断線など）
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    thermo.clearFault();
  }

  // ==========================================
  // 4. 【追加】ロボットシェフの自動塩分調整ロジック
  // ==========================================
  // 伝導率の電圧が0.5Vを下回ったら「塩が足りない」と判断する例
  if (ecVoltage < 0.5) {
    Serial.println(">>> 塩が足りません！ソルトミルを起動します...");
    
    // リレーをONにしてモーターを回す
    digitalWrite(RELAY_PIN, HIGH); 
    
    // 0.5秒間だけ挽く（一気に大量に入れないためのフィードバック制御）
    delay(500); 
    
    // リレーをOFFにしてモーターを止める
    digitalWrite(RELAY_PIN, LOW); 
    
    Serial.println(">>> 塩を投入しました。溶け切るまで待機します。");
    // 塩が水に溶けて伝導率センサーに反映されるまで3秒待つ（連続投入を防ぐ）
    delay(3000); 
    
  } else {
    Serial.println(">>> 十分な塩加減です。");
  }

  Serial.println("-------------------------");
  
  // 通常のループ待機時間
  delay(1000); 
}