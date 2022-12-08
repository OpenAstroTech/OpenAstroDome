// LimitSwitch.h

#ifndef _LIMITSWITCH_h
#define _LIMITSWITCH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Motor.h"

class LimitSwitch
	{
	enum ShutterState
	{
		OPENING,
		CLOSING
	};
	
	public:
		LimitSwitch(Motor* stepper, uint8_t limit);
		bool isOpen() const;
		bool isClosed() const;
		void init() const;
		void onMotorStopped();

	private:
		uint8_t limitPin;
		static volatile bool closeTriggered;
		static Motor* motor;
		static void onLimitReached();
		static void onCloseLimitReached();
	};

#endif

