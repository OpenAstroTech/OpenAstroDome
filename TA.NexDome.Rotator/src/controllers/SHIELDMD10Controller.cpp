#include "SHIELDMD10Controller.h"

namespace SHIELDMD10
{
SHIELDMD10::SHIELDMD10(uint8_t PWM, uint8_t DIR)
{
    _PWM = PWM;
    _DIR = DIR;
    pinMode(_PWM, OUTPUT);
    pinMode(_DIR, OUTPUT);
}

void SHIELDMD10::TurnRight(uint8_t pwm)
{
    digitalWrite(_DIR, 0);
    analogWrite(_PWM, pwm);
}

void SHIELDMD10::TurnLeft(uint8_t pwm)
{
    digitalWrite(_DIR, 1);
    analogWrite(_PWM, pwm);
}

void SHIELDMD10::Stop()
{
    analogWrite(_PWM, LOW);
}

Motor::Motor(uint8_t n)
{
    _isRunning = false;
}

void Motor::setup()
{
    motorController = new SHIELDMD10(MOTOR_DIRECTION_PIN, MOTOR_PWM_PIN);
}

void Motor::run(int dir, int pwm)  // dir is 0 or 1
{
    // 0 = Clockwise
    // 1 = Counter Clockwise
    switch (dir)
    {
        case 0:
            motorController->TurnRight(pwm);
            _isRunning = true;
            break;

        case 1:
            motorController->TurnLeft(pwm);
            _isRunning = true;
            break;
        default:
            break;
    }
}

void Motor::stop()
{
    motorController->Stop();
    _isRunning = false;
}

void Motor::brake()
{
    motorController->Stop();
    _isRunning = false;
}

bool Motor::isRunning()
{
    return _isRunning;
}

int Motor::readCurrent()
{
    return 0;
}
}  // namespace SHIELDMD10