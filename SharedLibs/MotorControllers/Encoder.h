#pragma once

#include <Arduino.h>

class Encoder
{
  public:
    Encoder(volatile int32_t *currentPosition, uint8_t ENCA, uint8_t ENCB);
    static void updateEncoder();
  
  private:
    static uint8_t _ENCA;
    static uint8_t _ENCB;
    static volatile int32_t *_currentPosition;
};
