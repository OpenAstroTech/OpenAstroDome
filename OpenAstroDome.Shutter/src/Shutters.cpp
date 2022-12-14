#include "Shutters.h"

Shutters::Shutters(MotorSettings& settings)
{
    configuration = &settings;
    leftStepper = new AccelStepper(AccelStepper::DRIVER, LEFT_MOTOR_STEP_PIN, LEFT_MOTOR_DIRECTION_PIN, MOTOR_ENABLE_PIN);
    rightStepper = new AccelStepper(AccelStepper::DRIVER, RIGHT_MOTOR_STEP_PIN, RIGHT_MOTOR_DIRECTION_PIN, MOTOR_ENABLE_PIN);
    rightStopHandler = nullptr;
    leftStopHandler = nullptr;
}


void Shutters::init()
{
    rightStepper->setPinsInverted(true, false, true);
    leftStepper->setPinsInverted(false, false, false);

    rightNewPosition = rightStepper->currentPosition();
    leftNewPosition = leftStepper->currentPosition();

    leftStepper->setMaxSpeed(4000.0);
    rightStepper->setMaxSpeed(4000.0);

    leftStepper->setAcceleration(2000.0);
    rightStepper->setAcceleration(2000.0);

    energizeMotor();
}

void Shutters::energizeMotor() const
{
    leftStepper->enableOutputs();
    rightStepper->enableOutputs();
}

void Shutters::releaseMotor()
{
    leftStepper->disableOutputs();
    rightStepper->disableOutputs();
	//digitalWrite(MOTOR_ENABLE_PIN, HIGH);	// active low
	//digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);		// active high, so ensure we are not commanding a step.
	//digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);		// active high, so ensure we are not commanding a step.
}

void Shutters::registerRightStopHandler(RightStopHandler handler)
{
	this->rightStopHandler = handler;
}

void Shutters::registerLeftStopHandler(LeftStopHandler handler)
{
	this->leftStopHandler = handler;
}

int32_t Shutters::limitOfTravel() const
{
	return configuration->maxPosition;
	//return MaxStepPosition;
}

void Shutters::SetLimitOfTravel(uint32_t limit)
{
    configuration->maxPosition = limit;
}

void Shutters::setMaximumSpeed(uint16_t speed)
{
    configuration->maxSpeed = speed;
}

uint16_t Shutters::getMaximumSpeed()
{
    return configuration->maxSpeed;
}

uint16_t Shutters::getMinimumSpeed()
{
	return minSpeed;
}

void Shutters::setRampTime(uint16_t milliseconds)
{
    configuration->rampTimeMilliseconds = milliseconds;
}

void Shutters::hardStop()
{
    rightStepper->stop();
    leftStepper->stop();
    rightNewPosition = rightStepper->currentPosition();
    leftNewPosition = leftStepper->currentPosition();
    leftCanRun = false;
    rightCanRun = false;

    if (rightStopHandler != nullptr)
    {
		rightStopHandler();
	}

    if (leftStopHandler != nullptr)
    {
		leftStopHandler();
	}
}

void Shutters::open()
{   
    leftNewPosition = leftStepper->currentPosition()+MaxStepPosition;
    rightNewPosition = rightStepper->currentPosition()+MaxStepPosition;
    leftStepper->setSpeed(4000);
    rightStepper->setSpeed(4000);
    leftCanRun = true;
    rightCanRun = true;
}

void Shutters::close()
{
    leftNewPosition = leftStepper->currentPosition()-MaxStepPosition;
    rightNewPosition = rightStepper->currentPosition()-MaxStepPosition;
    leftStepper->setSpeed(4000);
    rightStepper->setSpeed(4000);
    leftCanRun = true;
    rightCanRun = true;
}

void Shutters::stop()
{
    rightStepper->stop();
    leftStepper->stop();
    leftStepper->setSpeed(0);
    rightStepper->setSpeed(0);
    rightNewPosition = rightStepper->currentPosition();
    leftNewPosition = leftStepper->currentPosition();
    leftCanRun = false;
    rightCanRun = false;
}

void Shutters::stopOpenRight()
{
    rightStepper->stop();
    rightStepper->setSpeed(0); 
    rightNewPosition = rightStepper->currentPosition();
    configuration->maxPosition = rightNewPosition;
    rightCanRun = false;
}

void Shutters::stopClosedRight()
{
    rightStepper->stop();
    rightStepper->setSpeed(0);
    rightStepper->setCurrentPosition(0);
    rightNewPosition = rightStepper->currentPosition();
    rightCanRun = false;

    if (rightStopHandler != nullptr)
    {
		rightStopHandler();
	}
}

void Shutters::stopOpenLeft()
{
    leftStepper->stop();
    leftStepper->setSpeed(0);
    leftNewPosition = leftStepper->currentPosition();
    leftCanRun = false;
}

void Shutters::stopClosedLeft()
{
    leftStepper->stop();
    leftStepper->setSpeed(0);
    leftStepper->setCurrentPosition(0);
    leftNewPosition = leftStepper->currentPosition();
    leftCanRun = false;

    if (leftStopHandler != nullptr)
    {
		leftStopHandler();
	}
}

void Shutters::loop()
{
    if(leftCanRun)
    {
        if(leftStepper->currentPosition() != leftNewPosition)
        {
            leftStepper->moveTo(leftNewPosition);
            leftStepper->run();
        }
    }
    else
    {
        leftStepper->setCurrentPosition(leftNewPosition);
    }
    


    if(rightCanRun)
    {
        if(rightStepper->currentPosition() != rightNewPosition)
        {
            rightStepper->moveTo(rightNewPosition);
            rightStepper->run();
        }
    }
    else {
        rightStepper->setCurrentPosition(rightNewPosition);
    }
    
    
    
}