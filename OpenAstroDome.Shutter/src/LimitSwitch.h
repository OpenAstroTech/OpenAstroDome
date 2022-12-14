
// LimitSwitch.h

#ifndef _LIMITSWITCH_h
#define _LIMITSWITCH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <pcint.h>
#include "Motor.h"
#include "Shutters.h"
#include <AccelStepper.h>

class LimitSwitchRight
	{
	public:
		LimitSwitchRight(Shutters* shutters, uint8_t openLimit, uint8_t closeLimit);
		bool isOpen() const;
		bool isClosed() const;
		void init() const;
		void onMotorStopped();

	private:
		const uint8_t openLimitPin;
		const uint8_t closedLimitPin;
		static volatile bool closeTriggered;
		static Shutters* shutter;
		static AccelStepper* stepper;
		static void onOpenLimitReached();
		static void onCloseLimitReached();
	};

class LimitSwitchLeft
	{
	public:
		LimitSwitchLeft(Shutters* shutters, uint8_t openLimit, uint8_t closeLimit);
		bool isOpen() const;
		bool isClosed() const;
		void init() const;
		void onMotorStopped();

	private:
		const uint8_t openLimitPin;
		const uint8_t closedLimitPin;
		static volatile bool closeTriggered;
		static Shutters* shutter;
		static AccelStepper* stepper;
		static void onOpenLimitReached();
		static void onCloseLimitReached();
	};

#endif