#include "SHIELDMD10Controller.h"

SHIELDMD10::SHIELDMD10()
{
    _isRunning = false;
    halt = true;
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
}

void SHIELDMD10::run(int dir, int pwm)  // dir is 0 or 1
{
    digitalWrite(MOTOR_DIRECTION_PIN, dir);
    analogWrite(MOTOR_PWM_PIN, pwm * !halt);
    _isRunning = true;
}
void SHIELDMD10::stop()
{
    halt = true;
    analogWrite(MOTOR_PWM_PIN, LOW);
    _isRunning = false;
}

void SHIELDMD10::brake()
{
    halt = true;
    analogWrite(MOTOR_PWM_PIN, LOW);
    _isRunning = false;
}

bool SHIELDMD10::isRunning()
{
    return _isRunning;
}

int SHIELDMD10::readCurrent()
{
    return 0;
}
