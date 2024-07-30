#include "BTS7960Controller.h"

BTS7960::BTS7960()
{
    _R_PWM = MOTOR_PWM_PIN_R;
    _L_PWM = MOTOR_PWM_PIN_L;
    _L_EN  = MOTOR_ENABLE_PIN_L;
    _R_EN  = MOTOR_ENABLE_PIN_R;
    pinMode(_R_PWM, OUTPUT);
    pinMode(_L_PWM, OUTPUT);
    pinMode(_L_EN, OUTPUT);
    pinMode(_R_EN, OUTPUT);
    digitalWrite(_L_EN, HIGH);
    digitalWrite(_R_EN, HIGH);
    _isRunning = false;
    halt = true;
}

void BTS7960::run(int dir, int pwm)
{

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

void BTS7960::stop()
{
    halt = true;
    analogWrite(_L_PWM, LOW);
    analogWrite(_R_PWM, LOW);
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