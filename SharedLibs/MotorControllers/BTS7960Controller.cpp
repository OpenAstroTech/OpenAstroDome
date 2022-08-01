#include "BTS7960Controller.h"

namespace BTS7960
{
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
}

void BTS7960::TurnRight(uint8_t pwm)
{
    analogWrite(_L_PWM, 0);
    delayMicroseconds(100);
    analogWrite(_R_PWM, pwm);
}

void BTS7960::TurnLeft(uint8_t pwm)
{
    analogWrite(_R_PWM, 0);
    delayMicroseconds(100);
    analogWrite(_L_PWM, pwm);
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

void BTS7960::Stop()
{
    analogWrite(_L_PWM, LOW);
    analogWrite(_R_PWM, LOW);
}

Motor::Motor()
{
    _isRunning = false;
}

void Motor::setup()
{
    motorController = new BTS7960(MOTOR_ENABLE_PIN_R, MOTOR_ENABLE_PIN_L, MOTOR_PWM_PIN_R, MOTOR_PWM_PIN_L);
    motorController->Enable();
}

void Motor::run(int dir, int pwm)  // dir is 0 or 1
{
    // 0 = Clockwise
    // 1 = Counter Clockwise
    switch (dir)
    {
        case 0:
            motorController->Enable();
            motorController->TurnRight(pwm);
            _isRunning = true;
            break;

        case 1:
            motorController->Enable();
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
    motorController->Disable();
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
}