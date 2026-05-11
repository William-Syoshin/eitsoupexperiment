#include <OneWire.h>
#include <DallasTemperature.h>

// データ線を繋いだピン番号 (27)
const int oneWireBus = 27;     

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void setup() {
  // PCとの通信速度を115200に設定
  Serial.begin(115200);
  sensors.begin();
}

void loop() {
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  
  // 温度の数値だけをPCに送信
  Serial.println(temperatureC);
  
  // 2秒ごとに計測
  delay(2000);
}