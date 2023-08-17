#include "BTS7960Controller.h"

BTS7960::BTS7960(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM)
{
    _R_PWM = R_PWM;
    _L_PWM = L_PWM;
    _L_EN  = L_EN;
    _R_EN  = R_EN;
    pinMode(_R_PWM, OUTPUT);
    pinMode(_L_PWM, OUTPUT);
    pinMode(_L_EN, OUTPUT);
    pinMode(_R_EN, OUTPUT);
    _isRunning = false;
    halt = true;
}

void BTS7960::run(int dir, int pwm)
{
    Enable();
    if (dir = 0){
        analogWrite(_L_PWM, 0);
        delayMicroseconds(100);
        analogWrite(_R_PWM, pwm * !halt);
    } else if (dir = 1){
        analogWrite(_R_PWM, 0);
        delayMicroseconds(100);
        analogWrite(_L_PWM, pwm * !halt);
    }
    _isRunning = true;
}

void BTS7960::Enable()
{
    digitalWrite(_L_EN, 1);
    //digitalWrite(_R_EN, 1);
    if (_R_EN != 0)
        digitalWrite(_R_EN, HIGH);
}

void BTS7960::Disable()
{
    digitalWrite(_L_EN, 0);
    //digitalWrite(_R_EN,0);
    if (_R_EN != 0)
        digitalWrite(_R_EN, LOW);
}

void BTS7960::stop()
{
    halt = true;
    analogWrite(_L_PWM, LOW);
    analogWrite(_R_PWM, LOW);
    Disable();
    _isRunning = false;
}

bool BTS7960::isRunning()
{
    return _isRunning;
}

int BTS7960::readCurrent()
{
    return 0;
}