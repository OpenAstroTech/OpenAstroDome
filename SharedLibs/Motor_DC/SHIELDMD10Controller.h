#pragma once

#include <Arduino.h>
#include "OpenAstroDome.h"

class SHIELDMD10
{
  public:
    SHIELDMD10();
    void run(int dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
    bool halt;

  private:
    bool _isRunning;
};
