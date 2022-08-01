#pragma once

#include <Arduino.h>
#include "NexDome.h"

class Encoder
{
  public:
    Encoder(uint8_t ENCA, uint8_t ENCB);
    int32_t GetPosition();
    volatile int32_t encoderPosition; // specify encoder as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
    void updateEncoder();

  private:
    uint8_t _ENCA;
    uint8_t _ENCB;
};

void _readEncoder();
