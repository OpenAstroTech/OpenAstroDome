#pragma once

#include <Arduino.h>
#include "OpenAstroDome.h"

class BTS7960
{
  public:
    BTS7960();
    void run(int dir, int pwm);
    void stop();
    bool isRunning();
    int readCurrent();
    bool halt;

  private:
    uint8_t _L_EN;
    uint8_t _R_EN;
    uint8_t _L_PWM;
    uint8_t _R_PWM;
    bool _isRunning;
};
