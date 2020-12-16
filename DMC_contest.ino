/////////////////////
// DMC_tracer(M.Hirai)
// sample soft
// Remodeling by kobankid
// Rev.1.0
////////////////////


//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ここから編集NG↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

////////ハードウェアパラメータ/////////
//OLED 関係
#include <Wire.h>
#include <Adafruit_SSD1306.h>
//#include <MsTimer2.h>
#include <TimerOne.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// function declaration
void timer_interrupt(void);
void parameter_display(void);
void goal_lap_check(void);
void time_check(void);
void disp_lap_time(void);
void control(void);

// ピン設定
//LED
#define LED       13
//プッシュSW
#define SW_L      7//左
#define SW_R      8//右

//ラインセンサAD
#define LS_L2     3//外側左センサAD (黒　0->1023　白)
#define LS_L1     2//内側左センサAD (黒　0->1023　白)
#define LS_R1     1//内側右センサAD (黒　0->1023　白)
#define LS_R2     0//外側右センサAD (黒　0->1023　白)

//可変抵抗
#define VR_L      7//左可変抵抗AD(0->1023)
#define VR_R      6//右可変抵抗AD(0->1023)

#define MT_PWM_L  5//左モータPWM (停止 0 -> 255 早)
#define MT_DIR_L  3//左モータ回転方向 (バック 0 -> 1　前進)
#define MT_PWM_R  6//右モータPWM (停止 0 -> 255 早)
#define MT_DIR_R  2//右モータ回転方向 (前進 0 -> 1　バック)

//↑↑↑↑↑↑↑↑↑↑↑↑ここまで編集NG↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


//↓↓↓↓↓↓↓↓↓↓↓↓ここから編集OK↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

////////ソフトウェアパラメータ/////////
//#define DEBUG

#ifdef DEBUG
#define GET_CYCLE(label, a)          \
    do {                            \
        i_time_history[a] = label;  \
        time_history[a] = micros(); \
        if (a < TIME_HISTORY_NUM) { \
            a++;                    \
        } else {                    \
            a = 0;                  \
        }                           \
    } while(0)
#else
#define GET_CYCLE(label, a)
#endif

#define ABS(a)          (a >= 0 ? a : -a)
#define SIGN(a)         (a >= 0 ? 1 : -1)
#define MAX_CLIP(a, b)  (a >= b ? a = b : a)
#define ZERO_CLIP(a)    (a < 0  ? a = 0 : a)

// 定数設定---------------------------------------------
#define SW_OFF  HIGH
#define SW_ON   LOW
#define MT_FORWARD_L  HIGH
#define MT_REVERSE_L  LOW
#define MT_FORWARD_R  LOW
#define MT_REVERSE_R  HIGH

/* Log parameter */
#define TIME_HISTORY_NUM 10

/* Control parameter */
#define CTRL_INTERVAL_MS (1)
#define CTRL_INTERVAL_US (500)
#define PG               (0.65)
#define DG               (2.00)
#define G1               (2)
#define G2               (3)
#define INNER_CTL_TH     (225)
#define ERR1_MAX         (300)
#define ERR2_MAX         (300)
#define ERR_MAX          ((G1 * ERR1_MAX) + (G2 * ERR2_MAX))
#define D_ERR_MAX        ((G1 + G2) * 150)
#define DEAD_ZONE_TH     (30 * (G1 + G2))
#define L1_SENS_OFS      (0)
#define L2_SENS_OFS      (30)
#define R1_SENS_OFS      (35)
#define R2_SENS_OFS      (0)
#define D_ERR_MAX        (300)


//---------------------------------------------------

//パラメータ設定---------------------------------------
#define THREAD_LINE 300 //IRセンサがラインを認識する閾値
#define LAP_NUM 2 //周回数(レースモード時)
//---------------------------------------------------

//変数宣言--------------------
long  line_sensor_l1;
long  line_sensor_r1;
long  line_sensor_l2;
long  line_sensor_r2;

long  vr_ad_l;
long  vr_ad_r;

long  vel_set_l;
long  vel_set_r;

unsigned long start_time;
unsigned long total_time;
unsigned long lap_time[LAP_NUM+1];
unsigned long boot_time;
byte n_lap = 0;


/* kobankid added variables */
unsigned long i_time_history[TIME_HISTORY_NUM];
unsigned long time_history[TIME_HISTORY_NUM];
unsigned long i_time = 0;
unsigned long elapse_time;
volatile unsigned int interrupt_flag = 0;
float p_err_old = 0;
unsigned long d_flag = 0;


//-----------------------------

//↑↑↑↑↑↑↑↑↑↑↑↑ここまで編集OK↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

//初期設定関数
void setup() {
  // put your setup code here, to run once:
  //　起動時最初に一回だけ走るプログラム

  //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ここから編集NG↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

  //IOポート設定
  pinMode(LED,OUTPUT);
  pinMode(SW_R,INPUT_PULLUP);
  pinMode(SW_L,INPUT_PULLUP);

  //シリアル通信を開始
  Serial.begin(9600);

  //OLED初期化
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x32
  Serial.println(F("SSD1306 allocation failed"));
  for (;;); // Don't proceed, loop forever
  }

  // Wire(Arduino-I2C)の初期化
  Wire.begin();

  //↑↑↑↑↑↑↑↑↑↑↑↑ここまで編集NG↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

  //↓↓↓↓↓↓↓↓↓↓↓↓ここから編集OK↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

  /* AD変換の時間を短くする */
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;

  //モータの回転方向を設定
  digitalWrite(MT_DIR_L,MT_FORWARD_L);//前進
  digitalWrite(MT_DIR_R,MT_FORWARD_R);//前進

  //機種名表示-----------------------
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.print(F("DMC TRACER"));
  display.display();
  // delay(2000);
  //--------------------------------

  i_time = 0;

  //右スイッチがONになるまでループ
  while(1)
  {
    if(digitalRead(SW_R)==SW_ON)
    {
      delay(2000);
      break;
    }
    //ラインセンサ値読み込み
    line_sensor_l1  = analogRead(LS_L1);
    line_sensor_l2  = analogRead(LS_L2);
    line_sensor_r1  = analogRead(LS_R1);
    line_sensor_r2  = analogRead(LS_R2);

    //ボリュームの読み込み
    vr_ad_l  = analogRead(VR_L);
    vr_ad_r  = analogRead(VR_R);

    //ボリュームのAD値(10bit)をPWM値(8bit)に変換
    vel_set_l = vr_ad_l * 255 / 1023;
    vel_set_r = vr_ad_r * 255 / 1023;

    //OLEDに各パラメータ値出力(デバッグ用)
    parameter_display();
  }

  Timer1.initialize(CTRL_INTERVAL_US);
  Timer1.attachInterrupt(timer_interrupt);

  //↑↑↑↑↑↑↑↑↑↑↑↑ここまで編集OK↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
}

void timer_interrupt(void) {
  // interrupt_flag = 1;
#ifdef INTERRUPT
  GET_CYCLE(1, i_time);
  control();
  GET_CYCLE(2, i_time);
#endif
  //Serial.println("interrupts");
}

void control(void)
{
  long p_err_abs, p_err_max;
  float p_err, d_err, err1, err2;
  long l_velocity, r_velocity;

  //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ここから編集OK↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

  /* Interrupt disable */
  noInterrupts();

  if (n_lap > LAP_NUM) {
      analogWrite(MT_PWM_R,0);
      analogWrite(MT_PWM_L,0);
      Timer1.stop();
      return;
  }

  /* Read sensor value */
  line_sensor_l1 = analogRead(LS_L1) + L1_SENS_OFS;
  line_sensor_r1 = analogRead(LS_R1) + R1_SENS_OFS;
  line_sensor_l2 = analogRead(LS_L2) + L2_SENS_OFS;
  line_sensor_r2 = analogRead(LS_R2) + R2_SENS_OFS;

  /* Error signal calcuration */
  err1        = (float)(G1 * (line_sensor_l1 - line_sensor_r1));
  err2        = (float)(G2 * (line_sensor_l2 - line_sensor_r2));
  p_err       = err1 + err2;
  p_err_abs   = ABS(p_err);

  if (d_flag == 1) {
      d_err = p_err - p_err_old;
  }

  d_flag = 1;
  p_err_old = p_err;

  /* within range ? */
  if (p_err_abs > DEAD_ZONE_TH) {
    /* calcurate the velocity */
    if (p_err < 0) {
        l_velocity = vel_set_l
            * (float)(1 - ((PG * p_err_abs / (float)ERR_MAX) + DG * (float)d_err / D_ERR_MAX));
        r_velocity = vel_set_r;
    } else {
        l_velocity = vel_set_l;
        r_velocity = vel_set_r
            * (float)(1 - ((PG * p_err_abs / (float)ERR_MAX) + DG * (float)d_err / D_ERR_MAX));
    }
  } else {
    l_velocity = vel_set_l;
    r_velocity = vel_set_r;
  }

  /* zero clip */
  ZERO_CLIP(l_velocity);
  ZERO_CLIP(r_velocity);

  /* set motor parameter */
  analogWrite(MT_PWM_L, l_velocity);
  analogWrite(MT_PWM_R, r_velocity);

  /* interrupt enable */
  interrupts();

#if 0
  Serial.print("p_err_abs = ");
  Serial.println(p_err_abs);

  Serial.print("line_sensor_l1 = ");
  Serial.println(line_sensor_l1);

  Serial.print("line_sensor_r1 = ");
  Serial.println(line_sensor_r1);

  Serial.print("line_sensor_l2 = ");
  Serial.println(line_sensor_l2);

  Serial.print("line_sensor_r2 = ");
  Serial.println(line_sensor_r2);

  Serial.print("err1 = ");
  Serial.println(err1);

  Serial.print("err2 = ");
  Serial.println(err2);

  Serial.print("l_velocity = ");
  Serial.println(l_velocity);

  Serial.print("r_velocity = ");
  Serial.println(r_velocity);
#endif

  return;
}


//メインループ
void loop() {

  // put your main code here, to run repeatedly:
  //繰り返し走るプログラム

  //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ここまで編集OK↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

  //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ここから編集NG↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
  boot_time = millis();
  goal_lap_check();//編集禁止
  time_check();//編集禁止
  //周回数が所定以上で停止
  if (n_lap > LAP_NUM){
    analogWrite(MT_PWM_R,0);
    analogWrite(MT_PWM_L,0);
    disp_lap_time();
    while(1)
    {
      //ゴール時無限ループ
    }
  }
  //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ここまで編集NG↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

#ifdef DEBUG
  noInterrupts();
  if (i_time == TIME_HISTORY_NUM) {
    for (int i = 0; i < i_time; i++) {
      if (i > 0) {
        elapse_time = time_history[i] - time_history[i-1];
        Serial.print("elapse_time[");
        Serial.print(i);
        Serial.print("]=");
        Serial.print(elapse_time);
        Serial.print("[us]");
        Serial.println(i_time_history[i]);
      }
    }
    i_time = 0;
  }
  interrupts();
#endif

}

//OLEDに各パラメータ値出力(デバッグ用)
void parameter_display(void){
    display.clearDisplay();
    display.setTextSize(1); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("Set L:"));
    display.print(vel_set_l);
    display.print(", ");
    display.print(F("Set R:"));
    display.print(vel_set_r);
    display.print("\n");
    display.print(F("L2:"));
    display.print(line_sensor_l2);
    display.print(F(", L1:"));
    display.print(line_sensor_l1);
    display.print("\n");
    display.print(F("R1:"));
    display.print(line_sensor_r1);
    display.print(F(", R2:"));
    display.print(line_sensor_r2);
    display.display();
    delay(5);
}


//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓以下関数編集禁止↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

//走行時間処理(編集禁止)
void time_check(void)
{
  static unsigned long boot_time_prev = 0;
  //レース中、タイマー計算
  if ((n_lap > 0) && (n_lap <= LAP_NUM)){
    total_time = boot_time - start_time;
  }
  //10msに1回ディスプレイ更新
  if(boot_time - boot_time_prev < 10)
  {
  }else{
    boot_time_prev = boot_time;
    disp_lap_time();
  }
}

//ゴール判定とラップカウンタ(編集禁止)
void goal_lap_check(void){

  bool flg_sgline;
  bool static flg_sgline_prev = 0;
  byte static disp_count;

  //ゴールラインのチェック処理----------------------------------------
  bool t_ls11 = (line_sensor_l1 < THREAD_LINE);
  bool t_lsr1 = (line_sensor_r1 < THREAD_LINE);
  bool t_lsl2 = (line_sensor_l2 < THREAD_LINE);
  bool t_lsr2 = (line_sensor_r2 < THREAD_LINE);

  flg_sgline = 0;
  //すべてのセンサでラインを検出したとき、ゴールラインの検出とする。
  if ((t_ls11 == 1) && (t_lsr1 == 1) && (t_lsl2 == 1) && (t_lsr2 == 1))
  {
    flg_sgline = 1;
  }
  bool t_line = LOW;
  //ゴールラインを検出　かつ　前回ゴールラインを検出していないとき
  //ゴールライン検出フラグをたて、ラップ数をインクリメント、ラップタイムを記録する。
  if ((flg_sgline == 1) && (flg_sgline_prev == 0)) {
    t_line = HIGH;
    if (n_lap == 0) {
      start_time = boot_time;
    } else if (n_lap <= LAP_NUM){
      if (n_lap > 1) {
        lap_time[n_lap] = boot_time - lap_time[n_lap - 1];
      } else {
        lap_time[n_lap] = boot_time - start_time;
      }
    }
    n_lap = n_lap + 1;
  }

  //前回ゴールライン検出フラグを更新
  flg_sgline_prev = flg_sgline;
}

void disp_lap_time(void)
{
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("T:"));

  float tt = ((float)total_time /1000.0);//時間の単位を変換[ms->s]

  display.print(tt,2);

  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(F("L:"));

  display.setCursor(40, 16);
  display.print(n_lap);
  display.display();
}
