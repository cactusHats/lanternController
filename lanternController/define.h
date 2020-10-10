//ポート設定
#define PIN_LED 3
#define PIN_RSHUT 5
#define PIN_FSHUT 6
#define PIN_LSHUT 7
#define PIN_BSHUT 8

//距離センサモード
//使用するモードのコメントアウトを解除
#define LONG_RANGE 1 //長距離モード
#define HIGH_SPEED 0 //高速モード
#define HIGH_ACCURACY 0 //高精度モード

//その他設定
#define NUM_SENSOR 4 //センサ数
#define RESPONSE_DISTANCE 200 //反応する距離[mm]
#define MAXIMUM_DISTANCE 2000 //最大距離[mm]

#define NUM_LED_DATA_SET 400 //ゆらぎデータの数
#define DELAY_LED_CONTROL 30 //ゆらぎデータを進める速度
#define BIAS_BRIGHTNESS 100 //照度決定の数式（バイアス）
#define DIAMETER_BRIGHTNESS 0.5 //照度決定の数式（倍率）
#define THRETHOLD_JUDGE 0.8 //人検知とする認識率0~1

#define DEBUG_MODE 0 //デバッグモード実行 YES=1
