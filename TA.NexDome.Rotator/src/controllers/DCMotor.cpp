#include "DCMotor.h"
#if MOTOR_BOARD == MOTOR_CONTROLLER_BTS7960
#include "BTS7960Controller.h"
#elif MOTOR_BOARD == MOTOR_CONTROLLER_SHIELDMD10
#include "SHIELDMD10Controller.h"
#endif

DCMotor::DCMotor(uint8_t stepPin, uint8_t enablePin, uint8_t directionPin, IStepGenerator& stepper, MotorSettings& settings)
{
	//the DCMotor class is made of an encoder and a PWM motor controller board. Inputs and functions are to emulate that of a stepper motor. Each "step" will increment the desired encoder position in the PID control loop
    _encoder = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B);
	configuration = &settings;
	currentVelocity = 0;
    targetPosition = 0;
	minSpeed = MIN_SPEED;
	initializeHardware();
	stopHandler = nullptr;
}

/*
The Step method will be called from an interrupt service routine, so
operations must be as short as possible and modify as little state as possible.
*/
void DCMotor::Step(bool state)
{
    //Each step should increment the targetPosition of the motor PID control loop. THe main program loop will need to repeatedly call a function to adjust the PWM based on the PID settings
}