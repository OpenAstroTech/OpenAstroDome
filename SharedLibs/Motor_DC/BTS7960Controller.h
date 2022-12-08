#pragma once

#include <Arduino.h>
#include "OpenAstroDome.h"

class BTS7960
{
  public:
    //BTS7960(uint8_t EN, uint8_t L_PWM, uint8_t R_PWM) : BTS7960(EN, 0, L_PWM, R_PWM);
    BTS7960();
    void Enable();
    void Disable();
    void run(int dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();

  private:
    bool _isRunning;
};
