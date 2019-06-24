/*
 * Provides interrupt processing for the home sensor.
 * 
 * The home sensor synchronizes the dome rotation step position when it is triggered.
 * When rotating in the positive direction, we synchronise to the falling edge.
 * When rotating in the negative direction, we synchronise to the rising edge.
 * When not rotating, activity is ignored.
 * 
 * Note: some fields have to be static because they are used during interrupts
 */

#include "NexDome.h"
#include "HomeSensor.h"

#pragma region static fields used within interrupt service routines
MicrosteppingMotor* HomeSensor::motor;
Home* HomeSensor::settings;
uint8_t HomeSensor::sensorPin;
volatile bool HomeSensor::state;
volatile bool HomeSensor::homingInProgress;
volatile bool HomeSensor::performingPostHomeSlew;
#pragma endregion

/*
 * Creates a new HomeSensor instance.
 * Note: sensorPin must correspond to a digital input pin that is valid for attaching interrupts.
 * Not all pins on all platforms support attaching interrupts.
 * Arduino Leonardo supports pins 0, 1, 2, 3, 7
 */
HomeSensor::HomeSensor(MicrosteppingMotor* stepper, Home* settings, const uint8_t sensorPin, CommandProcessor& processor)
	: commandProcessor(processor)
	{
	motor = stepper;
	HomeSensor::settings = settings;
	HomeSensor::sensorPin = sensorPin;
	}


/*
 * Triggered as an interrupt whenever the home sensor pin changes state.
 * Synchronizes the current motor stop position to the calibrated home position.
 */
void HomeSensor::onHomeSensorChanged()
	{
	state = digitalRead(sensorPin);
	if (homingInProgress)
		foundHome();
	//if (!motor->isMoving()) // Ignore state change if rotator not moving
	//	return;
	//const auto direction = motor->getCurrentDirection();
	//if ((state && direction < 0) || (!state && direction > 0))
	//	{
	//	// sync position on either the rising or falling edge, depending on rotation direction.
	//	motor->SetCurrentPosition(settings->position);
	//	if (homingInProgress)
	//		foundHome();
	//	}
	}

/*
 * Configures the hardware pin ready for use and attaches the interrupt.
 */
void HomeSensor::init()
	{
	pinMode(sensorPin, INPUT_PULLUP);
	state = digitalRead(sensorPin);
	homingInProgress = false;
	performingPostHomeSlew = false;
	attachInterrupt(digitalPinToInterrupt(sensorPin), onHomeSensorChanged, CHANGE);
	}


bool HomeSensor::atHome()
	{
	return !state && !homingInProgress && !performingPostHomeSlew;
	}

/*
 * Rotates up to 2 full rotations clockwise while attempting to detect the home sensor.
 */
void HomeSensor::findHome(int direction)
	{
	homingInProgress = true;
	performingPostHomeSlew = false;
	const auto distance = 2 * settings->microstepsPerRotation;	// Allow 2 full rotations only
	motor->moveToPosition(distance);
	}

/*
 * Stops a homing operation in progress.
 */
void HomeSensor::cancelHoming()
	{
	performingPostHomeSlew = false;
	homingInProgress = false;
	if (motor->isMoving())
		motor->SoftStop();
	}

/*
 * Once the home sensor has been detected, we instruct the motor to soft-stop.
 * We also set the flaf performingPostHomeSlew.
 * At some point in the future, the onMotorStopped method will be called, which will
 * then initiate the final slew to return exactly to the home sensor position.
 */
void HomeSensor::foundHome()
	{
	std::cout << "foundHome" << std::endl;
	motor->SetCurrentPosition(settings->position);
	homingInProgress = false;
	performingPostHomeSlew = true;
	motor->SoftStop();
	}

void HomeSensor::onMotorStopped()
	{
	std::cout << "onMotorStopped" << std::endl;
	homingInProgress = false;
	if (performingPostHomeSlew)
		{
		performingPostHomeSlew = false;
		const auto target = commandProcessor.targetStepPosition(settings->position);
		std::cout << "Post slew from "
			<< motor->getCurrentPosition()
			<< " to " << target
			<< std::endl;
		motor->moveToPosition(target);
		}
	}
