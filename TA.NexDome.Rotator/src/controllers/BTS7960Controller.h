#pragma once

#include <Arduino.h>
#include "NexDome.h"

namespace BTS7960
{
class BTS7960
{
  public:
    BTS7960(uint8_t EN, uint8_t L_PWM, uint8_t R_PWM) : BTS7960(EN, 0, L_PWM, R_PWM)
    {
    }
    BTS7960(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM);
    void Enable();
    void Disable();

    void TurnLeft(uint8_t pwm);
    void TurnRight(uint8_t pwm);
    void Stop();

  private:
    uint8_t _L_EN;
    uint8_t _R_EN;
    uint8_t _L_PWM;
    uint8_t _R_PWM;
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
    BTS7960 *motorController;
};
}
