//
//
//

#include <ArduinoSTL.h>
#include "LimitSwitch.h"
#include "OpenAstroDome.h"


Motor* LimitSwitch::motor;
// volatile String LimitSwitch::shutterStatus; // static and volatile because accessed in ISR

LimitSwitch::LimitSwitch(Motor* stepper, uint8_t openLimit, uint8_t closeLimit)
	: openLimitPin(openLimit), closedLimitPin(closeLimit)
	{
	LimitSwitch::motor = stepper;
	LimitSwitch::shutterStatus = "UNKNOWN";
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
	return shutterStatus == "OPENING" ? true : false;
	}

bool LimitSwitch::isClosing() const
	{
	return shutterStatus == "CLOSING" ? true : false;
	}

void LimitSwitch::setShutterStatus(String status)
	{
		if (status == "OPENING" || status == "CLOSING" || status == "UNKNOWN")
			{
				shutterStatus = status;
			}
	}

void LimitSwitch::onCloseLimitReached()
	{
	motor->SoftStop();
	motor->SetCurrentPosition(0);
	shutterStatus = "CLOSED";
	/*if (closeTriggered)
		return;
	if (motor->getCurrentVelocity() < 0)
		{
		closeTriggered = true;
		motor->SetCurrentPosition(SHUTTER_LIMIT_STOPPING_DISTANCE);
		motor->moveToPosition(0);
		}*/
	}

void LimitSwitch::onOpenLimitReached()
	{
	motor->SoftStop();
	motor->SetCurrentPosition(SHUTTER_FULL_OPEN_DEFAULT);
	shutterStatus = "OPEN";
	/*if (motor->getCurrentVelocity() > 0)
		{
		const auto stopPosition = motor->getCurrentPosition() + SHUTTER_LIMIT_STOPPING_DISTANCE;
		motor->moveToPosition(stopPosition);
		//if (stopPosition < motor->limitOfTravel())
		//	motor->SetLimitOfTravel(stopPosition);
		}*/
	}

void LimitSwitch::onMotorStopped()
	{
	shutterStatus = "UNKNOWN";
	}

std::string LimitSwitch::loop()
	{
	if (isOpen()){
		if (shutterStatus == "OPENING" || shutterStatus == "UNKNOWN"){
			onOpenLimitReached();
			return "open";
		}
		else if(shutterStatus == "CLOSED"){
			onOpenLimitReached();
			return "open";
		}
	}
	else if (isClosed()){
		if (shutterStatus == "CLOSING" || shutterStatus == "UNKNOWN"){
			onCloseLimitReached();
			return "closed";
		}
		else if (shutterStatus == "OPEN"){
			onCloseLimitReached();
			return "closed";
		}
	}
	else if (!isClosed() && !isOpen()){
		if (shutterStatus == "OPEN" || shutterStatus == "CLOSED"){
			shutterStatus = "UNKNOWN";
			return "open";
		}
	}
	return "unknown";
	}

void LimitSwitch::init() const
	{
	pinMode(openLimitPin, INPUT_PULLUP);
	pinMode(closedLimitPin, INPUT_PULLUP);
	if (isOpen()){
		shutterStatus = "OPEN";
		motor->SetCurrentPosition(SHUTTER_FULL_OPEN_DEFAULT);
	} else if (isClosed()){
		shutterStatus = "CLOSED";
		motor->SetCurrentPosition(0);
	} else {
		shutterStatus = "UNKNOWN";
		motor->SetCurrentPosition(1000);
	}
	//attachInterrupt(digitalPinToInterrupt(openLimitPin), onOpenLimitReached, FALLING);
	//attachInterrupt(digitalPinToInterrupt(closedLimitPin), onCloseLimitReached, FALLING);
	}
