#pragma once

#include <Arduino.h>

class Encoder
{
  public:
    Encoder(volatile int32_t &currentPosition, uint8_t ENCA, uint8_t ENCB);
    static void updateEncoderA();
    static void updateEncoderB();
  
  private:
    inline static uint8_t _ENCA;
    inline static uint8_t _ENCB;
    inline static volatile int32_t *_currentPosition;
};
