void ledControl(int* ledControlData, int &elementNum, float illuminance) {
  //一定時間おきにLEDの明るさを更新
  if (millis() - timeThen > DELAY_LED_CONTROL) {
    int brightness = int((double(ledControlData[elementNum] * DIAMETER_BRIGHTNESS) + BIAS_BRIGHTNESS) * illuminance);
    analogWrite(PIN_LED, brightness);
    timeThen = millis();

    elementNum++;
    if (elementNum >= NUM_LED_DATA_SET) elementNum = 0;
  }
}

void getDistanceData(int* rawDistance) {
  //距離センサの値取得
  rawDistance[0] = sensorR.readRangeContinuousMillimeters();
  rawDistance[1] = sensorF.readRangeContinuousMillimeters();
  rawDistance[2] = sensorL.readRangeContinuousMillimeters();
  rawDistance[3] = sensorB.readRangeContinuousMillimeters();

#if DEBUG_MODE
  for (int i = 0; i < NUM_SENSOR; i++) {
    Serial.print(rawDistance[i]);
    Serial.print(" ");
  }
  Serial.println();
#endif
}

void fixIlluminance(int rawIlluminance, double &illuminanceRate) {
  //範囲内なら照度を更新
  if (0 <= rawIlluminance && rawIlluminance <= 100)
    illuminanceRate = rawIlluminance / 100.0; //0.0~1.0
}

void fixDistance(int* rawDistance, int* fixedDistance) {
  //最大距離以内なら値を更新
  for (int i = 0; i < NUM_SENSOR; i++) {
    if (rawDistance[i] < MAXIMUM_DISTANCE)
      fixedDistance[i] = rawDistance[i];
  }
}

int checkDistance(int* fixedDistance) {
  //4つのうち1つでも閾値以下なら1を返す
  for (int i = 0; i < NUM_SENSOR; i++) {
    if (rawDistance[i] < RESPONSE_DISTANCE)
      return 1;
  }
  return 0;
}

int setupSensorAddress() {
  //R
  pinMode(PIN_RSHUT, INPUT);
  delay(150);
  if (sensorR.init()) sensorR.init();
  else return 0;
  delay(100);
  sensorR.setAddress((uint8_t)23);
  sensorR.setTimeout(500);

  //F
  pinMode(PIN_FSHUT, INPUT);
  delay(150);
  if (sensorF.init()) sensorF.init();
  else return 0;
  delay(100);
  sensorF.setAddress((uint8_t)24);
  sensorF.setTimeout(500);

  //L
  pinMode(PIN_LSHUT, INPUT);
  delay(150);
  if (sensorL.init()) sensorL.init();
  else return 0;
  delay(100);
  sensorL.setAddress((uint8_t)25);
  sensorL.setTimeout(500);

  //B
  pinMode(PIN_BSHUT, INPUT);
  delay(150);
  if (sensorB.init()) sensorB.init();
  else return 0;
  delay(100);
  sensorB.setAddress((uint8_t)26);
  sensorB.setTimeout(500);

  return 1;
}

void setupSensorMode() {
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensorR.setSignalRateLimit(0.25);
  sensorF.setSignalRateLimit(0.25);
  sensorL.setSignalRateLimit(0.25);
  sensorB.setSignalRateLimit(0.25);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensorR.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensorR.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensorF.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensorF.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensorL.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensorL.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensorB.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensorB.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensorR.setMeasurementTimingBudget(20000);
  sensorF.setMeasurementTimingBudget(20000);
  sensorL.setMeasurementTimingBudget(20000);
  sensorB.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensorR.setMeasurementTimingBudget(200000);
  sensorF.setMeasurementTimingBudget(200000);
  sensorL.setMeasurementTimingBudget(200000);
  sensorB.setMeasurementTimingBudget(200000);
#endif
}

void error(String message) {
#if DEBUG_MODE
  Serial.print("Error Message : ");
  Serial.println(message);
#endif
  while (1);
}

void setupPinMode() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RSHUT, OUTPUT);
  pinMode(PIN_FSHUT, OUTPUT);
  pinMode(PIN_LSHUT, OUTPUT);
  pinMode(PIN_BSHUT, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_RSHUT, LOW);
  digitalWrite(PIN_FSHUT, LOW);
  digitalWrite(PIN_LSHUT, LOW);
  digitalWrite(PIN_BSHUT, LOW);
}
