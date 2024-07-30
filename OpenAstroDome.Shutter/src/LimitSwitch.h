// LimitSwitch.h

#ifndef _LIMITSWITCH_h
#define _LIMITSWITCH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Motor.h"

enum ShutterStatus
	{
	Unknown = 0,
	Open,
	Closed,
	Opening,
	Closing,
	};

class LimitSwitch
	{
	public:
		LimitSwitch(Motor* stepper, uint8_t openLimit, uint8_t closeLimit);
		bool isOpen() const;
		bool isClosed() const;
		void init() const;
		void onMotorStopped();
		bool isOpening() const;
		bool isClosing() const;
		void setShutterStatus(String status);
		void loop();
		static void setShutterStatus(ShutterStatus newStatus);
		ShutterStatus getShutterStatus();

	private:
		static ShutterStatus shutterStatus;
		uint8_t openLimitPin;
		uint8_t closedLimitPin;
		uint8_t movementDirection;
		//String shutterStatus;
		//static volatile bool closeTriggered;
		static Motor* motor;
		void onOpenLimitReached();
		void onCloseLimitReached();
	};

#endif

