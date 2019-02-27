// Motor.h
#ifndef MOTOR_H_
#define MOTOR_H_
#include <Arduino.h>

#define LEDC_TIMER_BIT  8
#define LEDC_BASE_FREQ  490
#define VALUE_MAX       255

class Motor{
public:
  Motor(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t channel);
  void cw(uint8_t pwm);
  void ccw(uint8_t pwm);
  void stop();

private:
  uint8_t _pin1, _pin2, _pin3, _channel;
};

#endif
