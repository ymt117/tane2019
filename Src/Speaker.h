// Speaker.h
#ifndef SPEAKER_H_
#define SPEAKER_H_
#include <Arduino.h>

#define mC 261.626
#define mD 293.665
#define mE 329.628
#define mF 349.228
#define mG 391.995
#define mA 440.000
#define mB 493.883

#define LEDC_CHANNEL_2    2
#define LEDC_TIMER_13_BIT 13
#define LEDC_BASE_FREQ    5000

class Speaker{
public:
  Speaker(uint8_t sp);
  void noTone(int pin);
  void tone(int pin, int freq);
  void beep();

private:
  uint8_t _sp;
};

#endif
