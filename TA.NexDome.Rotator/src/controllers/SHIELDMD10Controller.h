#pragma once

#include <Arduino.h>
#include "NexDome.h"

namespace SHIELDMD10
{
class SHIELDMD10
{
  public:
    SHIELDMD10(uint8_t PWM, uint8_t DIR);
    void TurnLeft(uint8_t pwm);
    void TurnRight(uint8_t pwm);
    void Stop();

  private:
    uint8_t _DIR;
    uint8_t _PWM;
};

class Motor
{
  public:
    Motor(uint8_t nmotor);
    void setup();
    void run(int dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();

  private:
    bool _isRunning;
    uint8_t _nmotor;
    SHIELDMD10 *motorController;
};
}