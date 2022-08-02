// Motor.h

#ifndef _DCMotor_h
#define _DCMotor_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include "WProgram.h"
#endif
#include "NexDome.h"
#include "Encoder.h"
#include "Motor.h"

#if MOTOR_BOARD == MOTOR_CONTROLLER_BTS7960
#include "BTS7960Controller.h"
#elif MOTOR_BOARD == MOTOR_CONTROLLER_SHIELDMD10
#include "SHIELDMD10Controller.h"
#endif

typedef void (*StopHandler) ();

struct PIDSettings
	{
		float DCMOTOR_kp;
		float DCMOTOR_ki;
		float DCMOTOR_kd;
	};

struct PWMSettings
	{
		float pwm;
		bool dir;
	};

class DCMotor : public Motor
	{
	public:
		DCMotor(uint8_t stepPin, uint8_t enablePin, uint8_t directionPin, MotorSettings& settings);
		virtual void Step(bool state) final;
		//void MoveAtVelocity(float stepsPerSecond);
		void energizeMotor() const;
		void releaseMotor();
		void registerStopHandler(StopHandler handler);
		void setRampTime(uint16_t milliseconds);
		virtual void hardStop();
		virtual void SoftStop();
		virtual void loop();
		void ComputeAcceleratedVelocity();
		virtual void moveToPosition(int32_t position);
		void SetCurrentPosition(int32_t position);
		void SetLimitOfTravel(uint32_t limit);
		void setMaximumSpeed(uint16_t speed);
		float getCurrentVelocity() const;
		virtual int32_t getCurrentPosition();
	int32_t midpointPosition() const;
	int32_t limitOfTravel() const;
	uint16_t getMaximumSpeed();
	uint16_t getMinimumSpeed();
		virtual bool isMoving();
		virtual int8_t getCurrentDirection();
		int32_t distanceToStop() const;
		MotorSettings* configuration;
		int32_t getTargetPosition();

	private:
		#if MOTOR_BOARD == MOTOR_CONTROLLER_BTS7960
			BTS7960::Motor* _rotator;
		#elif MOTOR_BOARD == MOTOR_CONTROLLER_SHIELDMD10
			SHIELDMD10* _rotator;
		#endif
		Encoder* _encoder;
		int direction = +1;
		int32_t targetPosition{};
		unsigned long startTime{};
		float startVelocity{}, currentVelocity, targetVelocity{}, currentAcceleration{};
		float minSpeed;
		float getAcceleratedVelocity() const;
		float getDeceleratedVelocity() const;
		float accelerationFromRampTime();
		StopHandler stopHandler;
		float positionError;
		float previousTime;
		float integralError;
		PWMSettings calcFromPID(int32_t currentPosition, PIDSettings PIDConstants);
		PIDSettings accelerationPID;
		PIDSettings runPID;
		PWMSettings pwm;
		unsigned long PIDtimer;
	};

// Motor Parameters (defaults)

#ifndef MIN_SPEED
#define MIN_SPEED					(250)	// Minimum speed that can be timed by the hardware timer
#endif

#ifndef MIN_RAMP_TIME
#define MIN_RAMP_TIME				(100)	// Minimum ramp up/down time in milliseconds
#endif

#endif