#pragma once

#include <Arduino.h>
#include "NexDome.h"

class SHIELDMD10
{
  public:
    SHIELDMD10();
    void run(int dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();

  private:
    bool _isRunning;
};
