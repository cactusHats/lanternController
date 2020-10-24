/*
   小人のすみか by odenki
   ・行灯のLEDのゆらぎ制御
   ・距離センサのデータ収集
   ・シリアル通信でデータを送受信
   programmed by Masashi Seki
*/

#include "define.h"
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensorR;
VL53L0X sensorF;
VL53L0X sensorL;
VL53L0X sensorB;

int ledControlData[] = {
  136, 148, 161, 172, 186, 198, 204, 205, 201, 191, 177, 159, 143, 131, 118, 106, 97, 97, 107, 122, 140, 158, 174, 181, 173, 153, 130, 115, 108, 107, 119, 137, 153, 164, 171, 176, 179, 182, 185, 191, 202, 213, 218, 221, 221, 219, 200, 174, 153, 145, 145, 149, 154, 163, 173, 182, 189, 196, 205, 215, 219, 223, 225, 227, 248, 224, 223, 220, 215, 214, 214, 218, 219, 221, 222, 223, 255, 255, 255, 255, 255, 251, 223, 221, 220, 221, 222, 224, 224, 223, 222, 220, 217, 218, 221, 222, 222, 221, 217, 213, 214, 217, 221, 223, 224, 223, 219, 222, 220, 216, 210, 193, 177, 165, 155, 150, 154, 165, 179, 189, 189, 176, 152, 134, 114, 98, 86, 76, 67, 66, 78, 112, 136, 147, 147, 135, 119, 108, 107, 120, 140, 149, 152, 153, 144, 128, 119, 124, 143, 160, 162, 147, 126, 122, 137, 153, 160, 167, 174, 180, 189, 195, 200, 201, 201, 196, 184, 170, 163, 166, 178, 193, 199, 194, 178, 163, 154, 150, 147, 140, 133, 124, 112, 103, 100, 108, 130, 149, 163, 171, 177, 184, 192, 202, 211, 213, 213, 205, 184, 169, 165, 172, 189, 202, 207, 203, 188, 172, 160, 152, 152, 163, 178, 189, 199, 206, 212, 214, 214, 214, 209, 190, 171, 159, 153, 153, 163, 179, 194, 206, 212, 215, 215, 212, 201, 189, 177, 166, 162, 163, 172, 186, 199, 209, 213, 215, 217, 219, 220, 220, 218, 216, 207, 186, 169, 166, 171, 182, 190, 189, 179, 163, 146, 126, 104, 84, 67, 57, 54, 62, 83, 102, 121, 134, 147, 162, 169, 170, 163, 148, 131, 113, 97, 89, 89, 106, 128, 142, 153, 166, 178, 184, 181, 170, 156, 139, 125, 115, 115, 128, 150, 172, 191, 207, 214, 218, 220, 220, 219, 216, 211, 198, 189, 187, 190, 199, 207, 212, 213, 210, 198, 185, 180, 184, 196, 199, 190, 175, 166, 168, 178, 187, 189, 186, 178, 168, 164, 164, 172, 187, 195, 195, 188, 175, 164, 158, 157, 161, 170, 180, 190, 200, 210, 214, 216, 218, 220, 221, 223, 223, 229, 222, 221, 223, 222, 220, 219, 216, 212, 210, 212, 215, 217, 219, 220, 222, 223, 220, 221, 223, 220, 219, 216, 213, 210, 205, 197, 187, 175, 162, 149, 137, 130, 126, 126, 132, 147, 145, 143, 140
};

int rawDistance[4] = {0};
int fixedDistance[4] = {0};
int elementNum = 0; //ゆらぎデータの要素数
int tgtIlluminance = 100; //Unityから送られる照度0~100%
int crtIlluminance = 0; //現在の照度;
bool humanDetected = false;
double illuminanceRate = 0.0; //修正後の照度
double record[20] = {0}; //平滑化用データ保持配列
double recognitionRate = 0.0; //平滑化後の認識率 0~1
int activeSensor[4] = {SENSOR_ACTIVE_B, SENSOR_ACTIVE_L, SENSOR_ACTIVE_F, SENSOR_ACTIVE_R};

void setup() {
  Serial.begin(9600);

  setupPinMode(); //ポート設定
  Wire.begin();

  if (!setupSensorAddress()) { //i2cのアドレス設定
#if DEBUG_MODE
    error("Failed to detect and initialize sensor!");
#endif
  }
  setupSensorMode(); //センサのモード設定
  startSensor(); //センサ計測開始
}

void loop() {

  getDistanceData(rawDistance); //距離センサの値取得
  fixDistance(rawDistance, fixedDistance); //外れ値の除外
  humanDetected = checkDistance(fixedDistance); //人の検知
  recognitionRate = smoothing(record, humanDetected); //平滑化処理

  //検知データの送信
  if (recognitionRate > THRETHOLD_JUDGE) {
    Serial.print(1);
    Serial.print("\t");
    Serial.println();
  }
  else {
    Serial.print(0);
    Serial.print("\t");
    Serial.println();
  }

  //照度データの受信
  if (Serial.available() > 0) {
    char tmp = Serial.read();
    int mode = tmp - '0'; //文字型から整数型に変換

    switch (mode) {
      case 0: tgtIlluminance = 0; break;
      case 1: tgtIlluminance = 100; break;
      default: break;
    }
  }
  crtIlluminance = targetGeneration(crtIlluminance, tgtIlluminance, DELAY_LED_CHANGE); //目標値の遷移
  fixIlluminance(crtIlluminance, illuminanceRate); //外れ値の除外
  ledControl(ledControlData, elementNum, illuminanceRate); //LEDの制御

  delay(10);
}
