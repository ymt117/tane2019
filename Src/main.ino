#include <Wire.h>
#include <LPS.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <TinyGPS++.h>
#include <Kalman.h>
#include "BluetoothSerial.h"
#include "Speaker.h"
#include "Motor.h"
#include "mySD.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//#define DEBUG
//#define BTSERIAL 
#define COUNT_NUM 100
#define ON 1
#define OFF 0
#define STOP 0
#define CW 1
#define CCW 2
#define FLIGHT_TIME 180000 // FLIGHT_TIME[ms] 
#define DEG_TO_RAD 0.017453292519943

LPS ps;
LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
LSM6 imu;
TinyGPSPlus gps;
HardwareSerial ss(2);
Kalman kalmanX;
Kalman kalmanY;
Speaker sp = Speaker(12);
Motor m1 = Motor(4, 13, 25, 0);
Motor m2 = Motor(27, 14, 26, 1);
mySD sd = mySD();
#ifdef BTSERIAL
  BluetoothSerial SerialBT;
#endif

// State transition of CanSat
enum MyState{
  State_calibrate = 0,
  State_launch,
  State_release,
  State_heat,
  State_comeback,
  State_goal,
  State_test
};
MyState s = State_calibrate;

// Prototype
void imuInit();
void calibrate();
float distance2goal();
float direction2goal();
float calcDirection();
void writeSD();

// Valiables
char report[80];
static const uint32_t GPSBaud = 9600;
static const uint8_t heat1 = 33;
static const uint8_t heat2 = 32;
static const uint8_t led1 = 2;
static const uint8_t led2 = 15;
static const uint8_t flightPin = 34;
/* 
 * g_lat, g_lng：目標地点の緯度，経度
 * 競技開始前に計測して入力しておくこと
*/
static const float g_lat = 31.568482;
static const float g_lng = 130.543202;
static const float threshold2goal = 5.0;

float accX, accY, accZ; // raw data 
float gyroX, gyroY, gyroZ; // raw data 

float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

// Timer Interrupt setting
hw_timer_t * h_timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//Velocity
float velX, velY, velZ = 0;
float comVelX, comVelY, comVelZ = 0;

float comAccX, comAccY, comAccZ = 0;  
float gravityX, gravityY, gravityZ = 0;

const float alpha = 0.8; // lowpassfilter

float magXoff = 1775.41058475;
float magYoff = 2081.76811925;
float magZoff = -15967.81415057;

// flag for judge landing
bool flag_flightPin = false;
bool flag_pressure = false;
bool flag_acceleration = false;
bool flag_timer = false;
uint32_t flag_timer_start = 0;
bool flag_fire = false;

enum motor_control{
  stop = 0,
  forward,
  right,
  left
};
motor_control mc = stop;

void setup(){
  Serial.begin(115200);
  Serial.println("Hello 100kinSAT!!!");
#ifdef BTSERIAL
  SerialBT.begin("Hello 100kinSAT111");
#endif
  sd.writeFile(SD, "/hello.txt", "Hello 100kinSAT\n");
  sd.writeFile(SD, "/log.csv", "millis,year,month,day,hour,minute,second,state,lat,lng,alt,ax,ay,az,comax,comay,comaz,gx,gy,gz,mx,my,mz,pre,tmp,roll,pitch,distance2goal,direction2goal,direction,flag_flightPin,flag_pressure,flag_acceleration,flag_timer,flag_fire,motor_control\n");
  Wire.begin();
  ss.begin(GPSBaud);

  pinMode(heat1, OUTPUT);
  pinMode(heat2, OUTPUT);
  digitalWrite(heat1, LOW);
  digitalWrite(heat2, LOW);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(flightPin, INPUT);

  // タイマー割り込みでmicroSD書き込みプログラムを実行する場合，リブートを繰り返してうまくいかないので使用しない
  // microSD書き込みプログラムの実行時間が長い（20ms~80ms程度）のが原因か
  //timerInit();
  imuInit();
  sp.beep();

  flag_timer_start = millis();
  s = State_launch;
}

void loop(){

  switch(s){
    case State_calibrate:
      calibrate();
      writeSD();
      s = State_test;
      break;
    case State_launch:
      led(led1, ON);
      judgeLanding();
      writeSD();
      if(flag_fire == true){
        led(led1, OFF);
        s = State_heat;
      }
      break;
    case State_release:
      break;
    case State_heat:
      led(led2, ON);
      heat(heat1, 5000);
      writeSD();
      delay(1000);
      heat(heat1, 5000);
      delay(5000);
      writeSD();
      heat(heat2, 5000);
      delay(1000);
      writeSD();
      heat(heat2, 5000);
      delay(1000);
      led(led2, OFF);
      writeSD();
      s = State_comeback;
      break;
    case State_comeback:
      m1.ccw(200);
      m2.cw(200);
      delay(5000);
      m1.stop();
      m2.stop();
      writeSD();
      s = State_goal;
      break;
    case State_goal:
      goal();
      break;
    case State_test:
      move2goal();
      //gps_test();
      break;
    default:
      // code block
      break;
  }
  
  delay(100);
}

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time or ISR
  portENTER_CRITICAL_ISR(&timerMux);

  // Interrupt function
  writeSD();
  Serial.println("interrupt");

  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use dititalRead/Write here if you want to toggle an output
}

void timerInit(){
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Refernce Manual for more info).
  h_timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(h_timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(h_timer, 1000000, true);

  // Start an alarm
  timerAlarmEnable(h_timer);
}

void imuInit(){
  if(!ps.init()){
    Serial.println("Failed to autdetect pressure sensor!");
    while(1);
  }
  ps.enableDefault();
  if(!mag.init()){
    Serial.println("Failed to detect and initialize magnetometer!");
    while(1);
  }
  mag.enableDefault();
  if(!imu.init()){
    Serial.println("Failed to detect and initialize IMU!");
    while(1);
  }
  imu.enableDefault();

  delay(100); // Wait for sensor to stablize

  imu.read();

  accX = imu.a.x;
  accY = imu.a.y;
  accZ = imu.a.z;
  gyroX = imu.g.x;
  gyroY = imu.g.y;
  gyroZ = imu.g.z;

  float roll = atan2(accY, accZ) * RAD_TO_DEG;
  float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void heat(uint8_t pin, int ms){
  if(pin == heat1 || pin == heat2){
    digitalWrite(pin, HIGH);
    delay(ms);
    digitalWrite(pin, LOW);
  } else{
    Serial.printf("%d is not set a pin for heating wires.\n", pin);
  }
}

void goal(){
  char buf[512];
  while(ss.available() > 0)
    gps.encode(ss.read());

  // Rename log.csv to log-year-month-day-hour-minute-second.csv
  String str = "";
  str += "/log-";
  str += gps.date.year();   str += "-";
  str += gps.date.month();  str += "-";
  str += gps.date.day();    str += "-";
  str += gps.time.hour();   str += "-";
  str += gps.time.minute(); str += "-";
  str += gps.time.second(); str += ".csv";

  int len = str.length();
  str.toCharArray(buf, len+1);

  sd.renameFile(SD, "/log.csv", buf);

  while(1){
    led(led1, ON);
    led(led2, OFF);
    delay(1000);
    led(led1, OFF);
    led(led2, ON);
    delay(1000);
    writeSD();
  }
}

void passedKalmanFilter(){
  imu.read();

  accX = imu.a.x;
  accY = imu.a.y;
  accZ = imu.a.z;
  gyroX = imu.g.x;
  gyroY = imu.g.y;
  gyroZ = imu.g.z;

  float dt = (float)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Calculate angle Kalman filter
  float roll = atan2(accY, accZ) * RAD_TO_DEG;
  float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  float gyroXrate = gyroX / 131.0; // Convert to deg/s
  float gyroYrate = gyroY / 131.0; // Convert to deg/s 

  if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)){
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else{
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }

  if(abs(kalAngleX) > 90){
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if(gyroXangle < -180 || gyroXangle > 180){
    gyroXangle = kalAngleX;
  }
  if(gyroYangle < -180 || gyroYangle > 180){
    gyroYangle = kalAngleY;
  }

  // Calculate gravitational acceleration
  gravityX = alpha * gravityX + (1 - alpha) * accX;
  gravityY = alpha * gravityY + (1 - alpha) * accY;
  gravityZ = alpha * gravityZ + (1 - alpha) * accZ;

  // Compensation acceleration
  comAccX = accX - gravityX;
  comAccY = accY - gravityY;
  comAccZ = accZ - gravityZ;

  if(abs(comAccX) < 100) comAccX = 0;
  if(abs(comAccY) < 100) comAccY = 0;
  if(abs(comAccZ) < 100) comAccZ = 0;

  velX = velX + comAccX * dt;
  velY = velY + comAccY * dt;
  velZ = velZ + comAccZ * dt;

#ifdef DEBUG
  Serial.print(comAccX); Serial.print("\t");
  Serial.print(comAccY); Serial.print("\t");
  Serial.print(comAccZ); Serial.print("\t");

  Serial.print(roll); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
#endif

  delay(2);
}

float distance2goal(){
  return sqrt(pow(g_lng - gps.location.lng(),2)+pow(g_lat - gps.location.lat(),2))*99096.44;
}

float direction2goal(){
  return atan2((gps.location.lng() - g_lng)*1.23,(gps.location.lat() - g_lat))*57.3+180;
}

float direction2oldPos(float o_lat, float o_lng){
  return atan2((gps.location.lng() - o_lng)*1.23,(gps.location.lat() - o_lat))*57.3+180;
}

void calibrate(){
  Serial.print("Calibrate ");
  led(led1, ON);
  for(int i=0; i<15000; i++){
    Serial.print(".");
    mag.read();
    // Acquire maximum and minimum values ​​from sensor
    running_min.x = _min(running_min.x, mag.m.x);
    running_min.y = _min(running_min.y, mag.m.y);
    running_min.z = _min(running_min.z, mag.m.z);

    running_max.x = _max(running_max.x, mag.m.x);
    running_max.y = _max(running_max.y, mag.m.y);
    running_max.z = _max(running_max.z, mag.m.z);
  }
  writeOffset();
  Serial.println(" done");
  led(led1, OFF);
  // Calculate offset value
  magXoff = (running_max.x + running_min.x) / 2;
  magYoff = (running_max.y + running_min.y) / 2;
  magZoff = (running_max.z + running_min.z) / 2;
}

// Use acceleration and magnetometer
float calcDirection(){
  passedKalmanFilter();
  mag.read();

  float roll = kalAngleX * DEG_TO_RAD;
  float pitch = kalAngleY * DEG_TO_RAD;

  float mag_x = mag.m.x;
  float mag_y = mag.m.y;
  float mag_z = mag.m.z;

  float numer = (mag_z-magZoff)*sin(roll)-(mag_y-magYoff)*cos(roll);
  float denom = (mag_x-magXoff)*cos(pitch)+(mag_y-magYoff)*sin(pitch)*sin(roll)+(mag_z-magZoff)*sin(pitch)*cos(roll);
  float theta = atan2(numer, denom) * RAD_TO_DEG;

  Serial.print(kalAngleX); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
  Serial.println(theta);

  return theta;
}

void judgeLanding(){
  // Use flight pin
  if(digitalRead(flightPin)){
    flag_flightPin = true;
  }
  // Use pressure

  // Use acceleration

  // Use timer
  if((millis() - flag_timer_start) > FLIGHT_TIME){
    flag_timer = true;
  }

  // Determine whether to ignite
  if(flag_flightPin == true && flag_timer == true){
    flag_fire = true;
  }
}

void move2goal(){
  float m_start = millis();
  while(ss.available() > 0)
    gps.encode(ss.read());
  
  writeSD();

  while(distance2goal() < 5){// Until the distance from the goal is 5 meters or less,
    float old_direction2goal = direction2goal();
    float o_lat = gps.location.lat();
    float o_lng = gps.location.lng();
    mc = forward;
    m_start = millis();
    while((millis() - m_start) > 20000){
      // forward
      m1.ccw(200);
      m2.cw(200);
      writeSD();
    }
    float tmp = old_direction2goal - direction2oldPos(o_lat, o_lng);
    if(tmp > 10){
      // Turn right
      mc = right;
      led(led1, ON);
      m_start = millis();
      while((millis() - m_start) > tmp*5.56){
        m1.cw(200);
        m2.cw(200);
      }
      led(led1, OFF);
      writeSD();
    } else if(tmp < -10){
      // Turn left
      mc = left;
      led(led2, ON);
      m_start = millis();
      while((millis() - m_start) > tmp*5.56){
        m1.ccw(200);
        m2.ccw(200);
      }
      led(led2, OFF);
      writeSD();
    } else{
      // do nothing
      led(led1, ON);
      led(led2, ON);
      delay(500);
      led(led1, OFF);
      led(led2, OFF);
      writeSD();
    }
  }

  mc = stop;
  m1.stop();
  m2.stop();
  s = State_goal;
  delay(20);
}

void led(uint8_t led, uint8_t state){
  if(state == ON) digitalWrite(led, HIGH);
  if(state == OFF) digitalWrite(led, LOW);
}

void writeOffset(){
  char buf[256];

  sd.writeFile(SD, "/mag_offset.txt", "max_x,max_y,max_z,min_x,min_y,min_z\n");

  String str = "";
  str += running_max.x;   str += ",";
  str += running_max.y;   str += ",";
  str += running_max.z;   str += ",";
  str += running_min.x;   str += ",";
  str += running_min.y;   str += ",";
  str += running_min.z;   str += ",";
  str += "\n";

  int len = str.length();
  str.toCharArray(buf, len+1);

  sd.appendFile(SD, "/mag_offset.txt", buf);
}

void writeSD(){
  char buf[1024];
  passedKalmanFilter();
  float theta = calcDirection();
  float pressure = ps.readPressureMillibars();
  float temperature = ps.readTemperatureC();
  while(ss.available() > 0)
    gps.encode(ss.read());

  String str = "";
  str += millis();                      str += ",";
  str += gps.date.year();               str += ",";
  str += gps.date.month();              str += ",";
  str += gps.date.day();                str += ",";
  str += gps.time.hour();               str += ",";
  str += gps.time.minute();             str += ",";
  str += gps.time.second();             str += ",";
  str += s;                             str += ",";
  str += String(gps.location.lat(), 6); str += ",";
  str += String(gps.location.lng(), 6); str += ",";
  str += gps.altitude.meters();         str += ",";
  str += imu.a.x;                       str += ",";
  str += imu.a.y;                       str += ",";
  str += imu.a.z;                       str += ",";
  str += comAccX;                       str += ",";
  str += comAccY;                       str += ",";
  str += comAccZ;                       str += ",";
  str += imu.g.x;                       str += ",";
  str += imu.g.y;                       str += ",";
  str += imu.g.z;                       str += ",";
  str += mag.m.x;                       str += ",";
  str += mag.m.y;                       str += ",";
  str += mag.m.z;                       str += ",";
  str += pressure;                      str += ",";
  str += temperature;                   str += ",";
  str += kalAngleX;                     str += ",";
  str += kalAngleY;                     str += ",";
  str += distance2goal();               str += ",";
  str += direction2goal();              str += ",";
  str += theta;                         str += ",";
  str += flag_flightPin;                str += ",";
  str += flag_pressure;                 str += ",";
  str += flag_acceleration;             str += ",";
  str += flag_timer;                    str += ",";
  str += flag_fire;                     str += ",";
  str += mc;
  str += "\n";

  int len = str.length();
  str.toCharArray(buf, len+1);

  sd.appendFile(SD, "/log.csv", buf);
}