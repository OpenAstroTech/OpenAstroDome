#pragma once
#include "MotorSettings.h"

class Motor
	{
	public:
		virtual int32_t getCurrentPosition();
		virtual void moveToPosition(int32_t position);
		virtual void setRampTime(uint16_t milliseconds);
		virtual void hardStop();
		virtual void SetCurrentPosition(int32_t position);
		virtual uint16_t getMinimumSpeed();
		virtual void setMaximumSpeed(uint16_t speed);
		virtual uint16_t getMaximumSpeed();
		virtual bool isMoving();
		virtual void SoftStop();
    };
