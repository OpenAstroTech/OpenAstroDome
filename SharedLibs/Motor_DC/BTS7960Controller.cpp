#include "BTS7960Controller.h"

BTS7960::BTS7960()
{
    pinMode(MOTOR_PWM_PIN_R, OUTPUT);
    pinMode(MOTOR_PWM_PIN_L, OUTPUT);
    pinMode(MOTOR_ENABLE_PIN_L, OUTPUT);
    pinMode(MOTOR_ENABLE_PIN_R, OUTPUT);
    _isRunning = false;
}

void BTS7960::run(int dir, int pwm)
{
    Enable();
    if (dir == 0){
        analogWrite(MOTOR_PWM_PIN_R, 0);
        delayMicroseconds(100);
        analogWrite(MOTOR_PWM_PIN_L, pwm);

    } else if (dir == 1){
        analogWrite(MOTOR_PWM_PIN_L, 0);
        delayMicroseconds(100);
        analogWrite(MOTOR_PWM_PIN_R, pwm);
    }
    _isRunning = true;
}

void BTS7960::Enable()
{
    digitalWrite(MOTOR_ENABLE_PIN_R, 1);
    if (MOTOR_ENABLE_PIN_L != 0)
        digitalWrite(MOTOR_ENABLE_PIN_L, HIGH);
}

void BTS7960::Disable()
{
    digitalWrite(MOTOR_ENABLE_PIN_R, 0);
    if (MOTOR_ENABLE_PIN_L != 0)
        digitalWrite(MOTOR_ENABLE_PIN_L, LOW);
}

void BTS7960::stop()
{
    analogWrite(MOTOR_PWM_PIN_L, LOW);
    analogWrite(MOTOR_PWM_PIN_R, LOW);
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