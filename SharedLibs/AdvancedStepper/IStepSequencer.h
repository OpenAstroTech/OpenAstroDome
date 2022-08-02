#pragma once
#include "../MotorControllers/Motor.h"

class IStepSequencer : public Motor
	{
	public:
		virtual void Step(bool state) = 0;
	};

