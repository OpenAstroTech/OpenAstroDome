// HomeSensor.h

#ifndef _HOMESENSOR_h
#define _HOMESENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <AdvancedStepper.h>
#include "CommandProcessor.h"

struct Home
	{
	int32_t position;	// Home sensor azimuth in microsteps
	unsigned int width;
	int32_t microstepsPerRotation;
	Home(int32_t stepPosition, unsigned width, int32_t circumferenceMicrosteps) 
		: position(stepPosition), width(width), microstepsPerRotation(circumferenceMicrosteps) {}
	};

class HomeSensor
	{
public:
	HomeSensor(MicrosteppingMotor* stepper, Home* settings, uint8_t sensorPin, CommandProcessor & processor);
	static void init();
	static bool atHome();
	static void findHome(int direction);
	static void cancelHoming();
	static void foundHome();
	void onMotorStopped();
private:
	static uint8_t sensorPin;
	static volatile bool state;
	static MicrosteppingMotor* motor;
	static Home* settings;
	CommandProcessor& commandProcessor;
	static void onHomeSensorChanged();
	static volatile bool homingInProgress;
	static volatile bool performingPostHomeSlew;
	};


#endif
