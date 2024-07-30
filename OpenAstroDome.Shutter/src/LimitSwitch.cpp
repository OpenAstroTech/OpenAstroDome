//
//
//

#include <ArduinoSTL.h>
#include "LimitSwitch.h"
#include "OpenAstroDome.h"

ShutterStatus LimitSwitch::shutterStatus;

Motor* LimitSwitch::motor;

LimitSwitch::LimitSwitch(Motor* stepper, uint8_t openLimit, uint8_t closeLimit)
	: openLimitPin(openLimit), closedLimitPin(closeLimit)
	{
	LimitSwitch::motor = stepper;
	setShutterStatus(Unknown);
	}

bool LimitSwitch::isOpen() const
	{
	return digitalRead(openLimitPin) == 0;
	}

bool LimitSwitch::isClosed() const
	{
	return digitalRead(closedLimitPin) == 0;
	}

bool LimitSwitch::isOpening() const
	{
	return shutterStatus == Opening;
	}

bool LimitSwitch::isClosing() const
	{
	return shutterStatus == Closing;
	}

void LimitSwitch::setShutterStatus(ShutterStatus newStatus)
{
	shutterStatus = newStatus;
}

ShutterStatus LimitSwitch::getShutterStatus()
{
	return shutterStatus;
}

void LimitSwitch::onCloseLimitReached()
	{
	motor->SoftStop();
	motor->SetCurrentPosition(0);
	setShutterStatus(Closed);
	}

void LimitSwitch::onOpenLimitReached()
	{
	motor->SoftStop();
	motor->SetCurrentPosition(SHUTTER_FULL_OPEN_DEFAULT);
	setShutterStatus(Open);
	}

void LimitSwitch::onMotorStopped()
	{
	setShutterStatus(Unknown);
	}

void LimitSwitch::loop()
	{
	if (isOpen()){
		if (shutterStatus != Closing){
			onOpenLimitReached();
		}
		motor->SetCurrentPosition(SHUTTER_FULL_OPEN_DEFAULT);
	}
	else if (isClosed()){
		if (shutterStatus != Opening){
			onCloseLimitReached();
		}
		motor->SetCurrentPosition(0);
	}
	else if (!isClosed() && !isOpen()){
		if (shutterStatus == Open || shutterStatus == Closed){
			setShutterStatus(Unknown);
		}
	}
	}

void LimitSwitch::init() const
	{
	pinMode(openLimitPin, INPUT_PULLUP);
	pinMode(closedLimitPin, INPUT_PULLUP);
	if (isOpen()){
		setShutterStatus(Open);
		motor->SetCurrentPosition(SHUTTER_FULL_OPEN_DEFAULT);
	} else if (isClosed()){
		setShutterStatus(Closed);
		motor->SetCurrentPosition(0);
	} else {
		setShutterStatus(Unknown);
		motor->SetCurrentPosition(1000);
	}
	}
