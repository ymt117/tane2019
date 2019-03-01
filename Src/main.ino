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

#define DEBUG
//#define BTSERIAL 
#define COUNT_NUM 100
#define ON 1
#define OFF 0

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
static bool feedgps();
float distance2goal();
float direction2goal();
float calcAzimuth();

// Valiables
char report[80];
static const uint32_t GPSBaud = 9600;
static const uint8_t heat1 = 33;
static const uint8_t heat2 = 32;
static const uint8_t led1 = 2;
static const uint8_t led2 = 15;
/* 
 * g_lat, g_lng：目標地点の緯度，経度
 * 競技開始前に計測して入力しておくこと
*/
static const float g_lat = 31.570648;
static const float g_lng = 130.545872;

float accX, accY, accZ; // raw data 
float gyroX, gyroY, gyroZ; // raw data 

float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

//Velocity
float velX, velY, velZ = 0;
float comVelX, comVelY, comVelZ = 0;

float comAccX, comAccY, comAccZ = 0;  
float gravityX, gravityY, gravityZ = 0;

const float alpha = 0.8; // lowpassfilter

float magXoff, magYoff, magZoff = 0;

void setup(){
  Serial.begin(115200);
  Serial.println("Hello 100kinSAT!!!");
#ifdef BTSERIAL
  SerialBT.begin("Hello 100kinSAT111");
#endif
  sd.writeFile(SD, "/hello.txt", "Hello 100kinSAT\n");
  sd.writeFile(SD, "/log.csv", "millis,year,month,day,hour,minute,second,state,lat,lng,ax,ay,az,comax,comay,comaz,gx,gy,gz,mx,my,mz,pre,tmp,roll,pitch,distance2goal,direction2goal,direction,m1,m2\n");
  Wire.begin();
  ss.begin(GPSBaud);

  pinMode(heat1, OUTPUT);
  pinMode(heat2, OUTPUT);
  digitalWrite(heat1, LOW);
  digitalWrite(heat2, LOW);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  imuInit();
  sp.beep();

  s = State_calibrate;
}

void loop(){
  uint32_t start = millis();
  uint32_t end = start;

  switch(s){
    case State_calibrate:
      calibrate();
      while(!feedgps()) Serial.print("#");
      Serial.println("GPS fixed!!!");
      s = State_test;
      break;
    case State_launch:
      break;
    case State_release:
      break;
    case State_heat:
      break;
    case State_comeback:
      break;
    case State_goal:
      break;
    case State_test:
      //passedKalmanFilter();
      //calcAzimuth();
      start = millis();
      writeSD();
      end = millis() - start;
      Serial.print(end);
      Serial.println(" ms");
      delay(1000);
      break;
    default:
      // code block
      break;
  }
  
  delay(100);
}

static bool feedgps(){
  while(ss.available()){
    if(gps.encode(ss.read())) return true;
  }
  return false;
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
  //Serial.print(velX); Serial.print("\t");
  //Serial.print(velY); Serial.print("\t");
  //Serial.print(velZ); Serial.print("\t");
  Serial.print(comAccX); Serial.print("\t");
  Serial.print(comAccY); Serial.print("\t");
  Serial.print(comAccZ); Serial.print("\t");

  Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

  //Serial.print("\r\n");

  delay(2);
#endif
}

float distance2goal(){
  return sqrt(pow(g_lng - gps.location.lng(),2)+pow(g_lat - gps.location.lat(),2))*99096.44;
}

float direction2goal(){
  return atan2((gps.location.lng() - g_lng)*1.23,(gps.location.lat() - g_lat))*57.3+180;
}

void calibrate(){
  Serial.print("Calibrate ");
  led(led1, ON);
  for(int i=0; i<10000; i++){
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
  Serial.println(" done");
  led(led1, OFF);
  // Calculate offset value
  magXoff = (running_max.x + running_min.x) / 2;
  magYoff = (running_max.y + running_min.y) / 2;
  magZoff = (running_max.z + running_min.z) / 2;
}

float calcAzimuth(){
  imu.read();
  mag.read();

  accX = imu.a.x;
  accY = imu.a.y;
  accZ = imu.a.z;

  float roll = atan2(accY, accZ);
  float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ));

  float numer = (mag.m.z-magZoff)*sin(roll)-(mag.m.y-magYoff)*cos(roll);
  float denom = (mag.m.x-magXoff)*cos(pitch)+(mag.m.y-magYoff)*sin(pitch)*sin(roll)+(mag.m.z-magZoff)*sin(pitch)*cos(roll);
  float theta = atan2(numer, denom) * RAD_TO_DEG;

#ifdef DEBUG
  //Serial.print(roll * RAD_TO_DEG);
  //Serial.print("\t");
  //Serial.print(pitch * RAD_TO_DEG);
  Serial.print("\t");
  Serial.println(theta);
#endif

  return theta;
}

void move2goal(){
  while(ss.available() > 0){
    char c = ss.read();
    gps.encode(c);
    if(gps.location.isUpdated()){

    #ifdef DEBUG
      Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
      Serial.print("\t");
      Serial.print("Lng: "); Serial.print(gps.location.lng(), 6);
      Serial.print("\t");
    #endif

      m1.stop();
      m2.stop();
      delay(1000);
      float direction = 0; // In the direction of CanSat
      for(int i=0; i<COUNT_NUM; i++){
        direction += calcAzimuth();
        delay(10);
      }
      direction = direction / COUNT_NUM; // Calculate direction average
      if(direction > 180) direction = 180;
      if(direction < -180) direction = -180;
      if(direction > 90){
        m1.cw(200);
        m2.cw(200);
      } else if(direction < -90){
        m1.ccw(200);
        m2.ccw(200);
      } else{
        m1.cw(200);
        m2.ccw(200);
        delay(750);
      }
      delay(250);
    }
  }
}

void led(uint8_t led, uint8_t state){
  if(state == ON) digitalWrite(led, HIGH);
  if(state == OFF) digitalWrite(led, LOW);
}

void writeSD(){
  char buf[1024];
  passedKalmanFilter();
  float theta = calcAzimuth();
  float pressure = ps.readPressureMillibars();
  float temperature = ps.readTemperatureC();

  String str = "";
  str += millis();            str += ",";
  str += gps.date.year();     str += ",";
  str += gps.date.month();    str += ",";
  str += gps.date.day();      str += ",";
  str += gps.time.hour();     str += ",";
  str += gps.time.minute();   str += ",";
  str += gps.time.second();   str += ",";
  str += s;                   str += ",";
  str += gps.location.lat();  str += ",";
  str += gps.location.lng();  str += ",";
  str += imu.a.x;             str += ",";
  str += imu.a.y;             str += ",";
  str += imu.a.z;             str += ",";
  str += comAccX;             str += ",";
  str += comAccY;             str += ",";
  str += comAccZ;             str += ",";
  str += imu.g.x;             str += ",";
  str += imu.g.y;             str += ",";
  str += imu.g.z;             str += ",";
  str += mag.m.x;             str += ",";
  str += mag.m.y;             str += ",";
  str += mag.m.z;             str += ",";
  str += pressure;            str += ",";
  str += temperature;         str += ",";
  str += kalAngleX;           str += ",";
  str += kalAngleY;           str += ",";
  str += distance2goal();     str += ",";
  str += direction2goal();    str += ",";
  str += theta;
  str += "\n";

  int len = str.length();
  str.toCharArray(buf, len+1);

  sd.appendFile(SD, "/log.csv", buf);
}

// Display IMU sensor value
void imu_test(){
  float pressure = ps.readPressureMillibars();
  mag.read();
  imu.read();

  Serial.print("P: ");
  Serial.print(pressure);
  snprintf(report, sizeof(report),
    " A: %5d %5d G: %5d %5d M: %5d %5d\n",
    imu.a.x, imu.a.y,
    imu.g.x, imu.g.y,
    mag.m.x, mag.m.y);
  Serial.print(report);
}
