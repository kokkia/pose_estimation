#include <MPU9250_asukiaaa.h>
#include <ArduinoJson.h>
//#include <ArduinoEigen.h>
//#include <MadgwickAHRS.h>
//#include "kal/kal.h"
#define DEBUG 1

#define G (9.8)//mps^2
#define Ts (0.01)//s
#define DEG2RAD (3.1415/180.0)
#define RAD2DEG (180.0/3.1415)

//9 axis sensor
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float roll, pitch, yaw;

//json
StaticJsonDocument<100> json_data;

//時間管理
double t = 0.0;//time
bool timer_flag = 0;

//timerの設定
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {//時間計測
  portENTER_CRITICAL_ISR(&timerMux);
  //control----------------------------------------//
  t += Ts;
  timer_flag = 1; 
  //-----------------------------------------------//
  portEXIT_CRITICAL_ISR(&timerMux);
}

//初期設定
void setup() {
  Serial.begin(115200);
  Serial.println("start");

  //9軸センサの設定
  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  
  //timer割り込み設定
  timer = timerBegin(0, 80, true);//プリスケーラ設定
  timerAttachInterrupt(timer, &onTimer, true);//割り込み関数指定
  timerAlarmWrite(timer, (int)(Ts*1000000), true);//Ts[s]ごとに割り込みが入るように設定
  timerAlarmEnable(timer);//有効化
 
  Serial.println("start");
  delay(1000);
  
}

//制御
void loop() {

  if(timer_flag){//制御周期
    timer_flag = 0;
    //9軸センサ取得
    mySensor.accelUpdate();
    mySensor.gyroUpdate();
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();

    json_data["aX"] = aX;
    json_data["aY"] = aY;
    json_data["aZ"] = aZ;
    json_data["gX"] = gX * DEG2RAD;
    json_data["gY"] = gY * DEG2RAD;
    json_data["gZ"] = gZ * DEG2RAD;
    serializeJson(json_data, Serial);
    Serial.println("");
    
#if DEBUG//グラフで確認用
//    Serial.print(gX);
//    Serial.print(",");
//    Serial.print(gY);
//    Serial.print(",");
//    Serial.print(gZ);
//    Serial.print(",");
//    Serial.print(roll);
//    Serial.print(",");
//    Serial.print(pitch);
//    Serial.print(",");
//    Serial.print(yaw);
//    Serial.print(",");
//    Serial.println();
#endif
  }//制御周期
  else{//その他の処理
  }
}
