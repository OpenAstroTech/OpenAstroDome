#pragma once
#include <Arduino.h>

struct MotorSettings
	{
	uint32_t maxPosition;				// limit of travel, in steps
	bool wraparound; 					// Motor can wraparound position from maxPosition back to zero (Eg. Rotator only, not shutter)
	volatile int32_t currentPosition;	// the current position (potentially updated by ISR)
	uint16_t rampTimeMilliseconds;		// milliseconds to ramp from minSpeed to maxSpeed
	uint16_t maxSpeed;					// maximum number of steps per second
	bool directionReversed;				// If true, reverses the rotation direction with respect to the step position
	bool useHoldingTorque;				// Apply holding torque after motor stops (otherwise de-energize the coils)
	};

template <typename T> int sgn(T val)
	{
	return (T(0) < val) - (val < T(0));
	}
