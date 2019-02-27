// Speaker.cpp
#include <Arduino.h>
#include "Speaker.h"

Speaker::Speaker(uint8_t sp){
  _sp = sp;
  pinMode(sp, OUTPUT);
}

void Speaker::noTone(int pin){
  ledcWriteTone(LEDC_CHANNEL_2, 0.0);
}

void Speaker::tone(int pin, int freq){
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(pin, LEDC_CHANNEL_2);
  ledcWriteTone(LEDC_CHANNEL_2, freq);
}

void Speaker::beep(){
  float mm[] = {mC*2, mD*2, mE*2, mF*2, mG*2, mA*2, mB*2, mC*4};
  for(int i = 0; i < sizeof(mm)/sizeof(float); i++){
    tone(_sp, mm[i]);
    delay(250);
  }
  noTone(_sp);
}
