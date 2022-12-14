//
//
//

#include <ArduinoSTL.h>
#include "LimitSwitch.h"
#include "OpenAstroDome.h"


AccelStepper* LimitSwitchRight::stepper;
AccelStepper* LimitSwitchLeft::stepper;
Shutters* LimitSwitchLeft::shutter;
Shutters* LimitSwitchRight::shutter;

volatile bool LimitSwitchRight::closeTriggered; // static and volatile because accessed in ISR
volatile bool LimitSwitchLeft::closeTriggered; // static and volatile because accessed in ISR

LimitSwitchRight::LimitSwitchRight(Shutters* shutters, uint8_t openLimit, uint8_t closeLimit)
	: openLimitPin(openLimit), closedLimitPin(closeLimit)
	{
	LimitSwitchRight::stepper = shutters->rightStepper;
	LimitSwitchRight::shutter = shutters;
	}

bool LimitSwitchRight::isOpen() const
	{
	return digitalRead(RIGHT_OPEN_LIMIT_SWITCH_PIN) == 0;
	}

bool LimitSwitchRight::isClosed() const
	{
	return digitalRead(RIGHT_CLOSED_SWITCH_PIN) == 0;
	}

void LimitSwitchRight::onCloseLimitReached()
	{
	if (closeTriggered)
		return;
	
	closeTriggered = true;
	//stepper->setCurrentPosition(SHUTTER_LIMIT_STOPPING_DISTANCE);
	//stepper->stop();
	shutter->stopClosedRight();

	/*
	if (motor->getCurrentVelocity() < 0)
		{
		closeTriggered = true;
		motor->SetCurrentPosition(SHUTTER_LIMIT_STOPPING_DISTANCE);
		motor->moveToPosition(0);
		}
	*/
	}

void LimitSwitchRight::onOpenLimitReached()
	{
		/*
		if (motor->getCurrentVelocity() > 0)
		{
		const auto stopPosition = motor->getCurrentPosition() + SHUTTER_LIMIT_STOPPING_DISTANCE;
		motor->moveToPosition(stopPosition);
		if (stopPosition < motor->limitOfTravel())
			motor->SetLimitOfTravel(stopPosition);
		}
		*/
		
		//const auto stopPosition = stepper->currentPosition() + SHUTTER_LIMIT_STOPPING_DISTANCE;
		//stepper->moveTo(stopPosition);
		//stepper->stop();
		/*
		if (stopPosition < motor->limitOfTravel())
			motor->SetLimitOfTravel(stopPosition);
		*/
		shutter->stopOpenRight();
		
	}

void LimitSwitchRight::onMotorStopped()
	{
	closeTriggered = false;
	}

void LimitSwitchRight::init() const
	{
	pinMode(RIGHT_OPEN_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(RIGHT_CLOSED_SWITCH_PIN, INPUT_PULLUP);
	closeTriggered = false;
	//attachInterrupt(digitalPinToInterrupt(openLimitPin), onOpenLimitReached, FALLING);
	//attachInterrupt(digitalPinToInterrupt(closedLimitPin), onCloseLimitReached, FALLING);
	PCattachInterrupt<RIGHT_OPEN_LIMIT_SWITCH_PIN>(onOpenLimitReached, FALLING);
	PCattachInterrupt<RIGHT_CLOSED_SWITCH_PIN>(onCloseLimitReached, FALLING);
	}




// LEFT
LimitSwitchLeft::LimitSwitchLeft(Shutters* shutters, uint8_t openLimit, uint8_t closeLimit)
	: openLimitPin(openLimit), closedLimitPin(closeLimit)
	{
	LimitSwitchLeft::stepper = shutters->leftStepper;
	LimitSwitchLeft::shutter = shutters;
	}

bool LimitSwitchLeft::isOpen() const
	{
	return digitalRead(LEFT_OPEN_LIMIT_SWITCH_PIN) == 0;
	}

bool LimitSwitchLeft::isClosed() const
	{
	return digitalRead(LEFT_CLOSED_LIMIT_SWITCH_PIN) == 0;
	}

void LimitSwitchLeft::onCloseLimitReached()
	{
	if (closeTriggered)
		return;
	/*
	if (motor->getCurrentVelocity() < 0)
		{
		closeTriggered = true;
		motor->SetCurrentPosition(SHUTTER_LIMIT_STOPPING_DISTANCE);
		motor->moveToPosition(0);
		}
	*/
		//stepper->setCurrentPosition(SHUTTER_LIMIT_STOPPING_DISTANCE);
		closeTriggered = true;
		shutter->stopClosedLeft();
	}

void LimitSwitchLeft::onOpenLimitReached()
	{
		/*
		if (motor->getCurrentVelocity() > 0)
		{
		const auto stopPosition = motor->getCurrentPosition() + SHUTTER_LIMIT_STOPPING_DISTANCE;
		motor->moveToPosition(stopPosition);
		if (stopPosition < motor->limitOfTravel())
			motor->SetLimitOfTravel(stopPosition);
		}
		*/
		//const auto stopPosition = stepper->currentPosition() + SHUTTER_LIMIT_STOPPING_DISTANCE;
		//stepper->moveTo(stopPosition);
		shutter->stopOpenLeft();
	}

void LimitSwitchLeft::onMotorStopped()
	{
	closeTriggered = false;
	}

void LimitSwitchLeft::init() const
	{
	pinMode(LEFT_OPEN_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(LEFT_CLOSED_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	closeTriggered = false;
	//attachInterrupt(digitalPinToInterrupt(openLimitPin), onOpenLimitReached, FALLING);
	//attachInterrupt(digitalPinToInterrupt(closedLimitPin), onCloseLimitReached, FALLING);
	PCattachInterrupt<LEFT_OPEN_LIMIT_SWITCH_PIN>(onOpenLimitReached, FALLING);
	PCattachInterrupt<LEFT_CLOSED_LIMIT_SWITCH_PIN>(onCloseLimitReached, FALLING);
	}