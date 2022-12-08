//
//
//

#include <ArduinoSTL.h>
#include "LimitSwitch.h"
#include "OpenAstroDome.h"


Motor* LimitSwitch::motor;
volatile bool LimitSwitch::closeTriggered; // static and volatile because accessed in ISR

LimitSwitch::LimitSwitch(Motor* stepper, uint8_t limit)
	: limitPin(limit)
	{
	LimitSwitch::motor = stepper;
	}

bool LimitSwitch::isOpen() const
	{
	return digitalRead(limitPin) == 0;
	}

bool LimitSwitch::isClosed() const
	{
	return digitalRead(limitPin) == 0;
	}

	/*
void LimitSwitch::onCloseLimitReached()
	{
	if (closeTriggered)
		return;
	if (motor->getCurrentVelocity() < 0)
		{
		closeTriggered = true;
		motor->SetCurrentPosition(SHUTTER_LIMIT_STOPPING_DISTANCE);
		motor->moveToPosition(0);
		}
	}
	*/

void LimitSwitch::onLimitReached()
	{
	if (motor->getCurrentVelocity() > 0)
		{
		const auto stopPosition = motor->getCurrentPosition() + SHUTTER_LIMIT_STOPPING_DISTANCE;
		motor->moveToPosition(stopPosition);
		//if (stopPosition < motor->limitOfTravel())
		//	motor->SetLimitOfTravel(stopPosition);
		}
	}

void LimitSwitch::onMotorStopped()
	{
	closeTriggered = false;
	}

void LimitSwitch::init() const
	{
	pinMode(limitPin, INPUT_PULLUP);
	//pinMode(closedLimitPin, INPUT_PULLUP);
	closeTriggered = false;
	attachInterrupt(digitalPinToInterrupt(limitPin), onLimitReached, FALLING);
	//attachInterrupt(digitalPinToInterrupt(closedLimitPin), onCloseLimitReached, FALLING);
	}
