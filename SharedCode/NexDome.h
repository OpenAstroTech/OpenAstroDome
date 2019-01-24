#pragma once

#define FIRMWARE_MAJOR_VERSION	(0)
#define FIRMWARE_MINOR_VERSION	(0)

// Motor Parameters
#define MICROSTEPS_PER_STEP			(8)		// Should match DIP switches on stepper driver
#define MIN_SPEED					(250)	// Minimum speed that can be timed by the hardware timer
#define MIN_RAMP_TIME				(100)	// Minimum ramp up/down time in milliseconds

// Motor 1, address "R" (rotator)
#define M1_STEP_PIN					(12)
#define M1_DIRECTION_PIN			(11)
#define M1_ENABLE_PIN				(10)
#define M1_MAX_POSITION				(440800 * MICROSTEPS_PER_STEP)
#define M1_RAMP_TIME				(2500)	// milliseconds to accelerate to full speed
#define M1_MAX_SPEED				(3200 * MICROSTEPS_PER_STEP)

#define SERIAL_RX_BUFFER_SIZE		(16)	// Receive buffer for PC/USB communications
