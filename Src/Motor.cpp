/*  Motor.cpp
main.ino
#include "Motor.h"
Motor m = Motor(pin1, pin2, pin3);
void setup(){}
void loop(){
  m.cw(255);
  delay(500);
  m.stop();
  delay(500);
}
 */
#include <Arduino.h>
#include "Motor.h"

Motor::Motor(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t channel){
  _pin1 = pin1;
  _pin2 = pin2;
  _pin3 = pin3;
  _channel = channel;

  ledcSetup(channel, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  
  ledcAttachPin(pin3, channel);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
}

void Motor::cw(uint8_t pwm){
  if(pwm > 255) pwm = 255;
  if(pwm < 0) pwm = 0;
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
  ledcWrite(_channel, pwm);
}

void Motor::ccw(uint8_t pwm){
  if(pwm > 255) pwm = 255;
  if(pwm < 0) pwm = 0;
  digitalWrite(_pin2, LOW);
  digitalWrite(_pin1, HIGH);
  ledcWrite(_channel, pwm);
}

void Motor::stop(){
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}
