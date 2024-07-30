#include "DCMotor.h"

DCMotor::DCMotor(uint8_t stepPin, uint8_t enablePin, uint8_t directionPin, MotorSettings& settings)
	{
		//the DCMotor class is made of an encoder and a PWM motor controller board. Inputs and functions are to emulate that of a stepper motor. Each "step" will increment the desired encoder position in the PID control loop

		#if MOTOR_BOARD == MOTOR_CONTROLLER_BTS7960
			_rotator = new BTS7960();
		#elif MOTOR_BOARD == MOTOR_CONTROLLER_SHIELDMD10
			_rotator = new SHIELDMD10();
		#endif
		configuration = &settings;
		currentVelocity = 0;
		targetPosition = configuration->currentPosition;
		positionError = 0;
		previousTime = 0;
		integralError = 0;
		minSpeed = MIN_SPEED;
		PID.DCMOTOR_kp = DCMOTOR_kp;
		PID.DCMOTOR_ki = DCMOTOR_ki;
		PID.DCMOTOR_kd = DCMOTOR_kd;
		_encoder = new Encoder(configuration->currentPosition, ENCODER_PIN_A, ENCODER_PIN_B);
	}

// Disables the motor coils (releases holding torque).
void DCMotor::releaseMotor()
	{
		_rotator->stop();
	}

void DCMotor::setWraparound(bool wraparound)
	{
		configuration->wraparound = wraparound;
	}

void DCMotor::setRampTime(uint16_t milliseconds)
	{
	configuration->rampTimeMilliseconds = milliseconds;
	}

/*
	Configures the motor to move to an absolute step position. Unless interrupted,
	the motor will commence stepping at minSpeed and will accelerate uniformly
	to maxSpeed. When nearing the target position, the motor will decelerate uniformly
	down to minSpeed and upon reaching the target position, will perform a hard stop.
	Note: for short moves the motor may never reach maxSpeed.
*/
void DCMotor::moveToPosition(int32_t position)
	{
	_rotator->halt = false;
	targetPosition = position;
	positionError = computePositionError(getCurrentPosition());
	direction = sgn(positionError);
	targetVelocity = configuration->maxSpeed * direction;
	currentAcceleration = accelerationFromRampTime() * direction;
	startTime = millis();
	previousTime = startTime;
	integralError = 0;
	if (abs(currentVelocity) < minSpeed)
		{
		// Starting from rest
		startVelocity = minSpeed * direction;
		currentVelocity = startVelocity;
		}
	else
		{
		// Starting with the motor already in motion
		startVelocity = currentVelocity;
		}
	}

/*
	Sets the motor's current step position. This does not cause any movement.
*/
void DCMotor::SetCurrentPosition(int32_t position)
	{
	configuration->currentPosition = position;
	}

/*
	Sets the limit of travel (maximum step position) of the motor.
*/
void DCMotor::SetLimitOfTravel(uint32_t limit)
	{
	configuration->maxPosition = limit;
	}

void DCMotor::setMaximumSpeed(uint16_t speed)
	{
	configuration->maxSpeed = speed;
	}

/*
	Gets the current motor velocity in steps per second.
*/
float DCMotor::getCurrentVelocity() const
	{
	return currentVelocity;
	}

/*
	Gets the current motor position in steps.
*/
int32_t DCMotor::getCurrentPosition()
	{
	return configuration->currentPosition;
	}

int32_t DCMotor::getTargetPosition()
	{
	return targetPosition;
	}

int32_t DCMotor::midpointPosition() const
	{
	return configuration->maxPosition / 2;
	}

int32_t DCMotor::limitOfTravel() const
	{
	return configuration->maxPosition;
	}

uint16_t DCMotor::getMaximumSpeed()
	{
	return configuration->maxSpeed;
	}

uint16_t DCMotor::getMinimumSpeed()
	{
	return minSpeed;
	}

bool DCMotor::isMoving()
	{
	return currentVelocity != 0;
	}

/*
	Computes the linear acceleration required to accelerate from rest to the maximum
	speed in the ramp time. The returned value is always positive.
	From v = u + at; since u is 0, v = at where t is the ramp time. Therefore, a = v/t.
*/
float DCMotor::accelerationFromRampTime()
	{
	const float rampTimeSeconds = float(configuration->rampTimeMilliseconds) / 1000.0;
	const float acceleration = float(configuration->maxSpeed) / rampTimeSeconds;
	return acceleration;
	}

/*
	Computes the theoretical accelerated velocity assuming uniform acceleration since start time.
	v = u + at
	u = startVelocity, a is acceleration, t is elapsed time since start
*/
float DCMotor::getAcceleratedVelocity() const
	{
	const float elapsedTime = (millis() - startTime) / 1000.0;
	const float acceleratedVelocity = startVelocity + currentAcceleration * elapsedTime; // v = u + at
	return acceleratedVelocity;
	}

/*
	Computes the maximum velocity that will still allow the motor to decelerate to minSpeed
	before reaching the target position. We do this by computing what the velocity would have been
	if we had started at the target position and accelerated back for n steps, then changing the sign of
	that velocity to match the current direction of travel.
	v² = u² + 2as
	u = minSpeed, a = |acceleration|, s = steps still to go
	|v| = √(u² + 2as) (positive root)
	maximum velocity = v * direction
*/
float DCMotor::getDeceleratedVelocity() const
	{
	const auto current = int32_t(configuration->currentPosition);
	const auto target = int32_t(targetPosition);
	const int32_t deltaSteps = target - current;
	const uint32_t stepsToGo = abs(deltaSteps);
	const auto acceleration = fabs(currentAcceleration);
	const auto uSquared = minSpeed * minSpeed;
	const auto vSquared = uSquared + 2.0 * acceleration * stepsToGo;
	const auto speed = sqrt(vSquared);
	const auto velocity = speed * direction;
	return velocity;
	}

/*
	Brings the motor to an immediate hard stop.
*/
void DCMotor::hardStop()
	{
	_rotator->stop();
	_rotator->halt = true;
	targetPosition = getCurrentPosition();
	currentAcceleration = 0;
	currentVelocity = 0;
	direction = 0;
	integralError = 0;
	positionError = 0;
	}
/*
 * Decelerate to a stop in the shortest distance allowed by the current acceleration.
 */
void DCMotor::SoftStop()
	{
	hardStop();
	}

void DCMotor::loop()
	{
	if (isMoving())
		computePID();
	}

/*
	Recomputes the current motor velocity. Call this from within the main loop.
*/
void DCMotor::computePID()
	{
	// Current Position
	int32_t currentPosition = getCurrentPosition();

	// Current Time
	long currentTime = micros();
	float deltaTime = ((float) (currentTime - previousTime))/( 1.0e6 );
	previousTime = currentTime;
	
	// Current Velocity
	const float accelerationCurve = getAcceleratedVelocity();
	const float decelerationCurve = getDeceleratedVelocity();
	const float computedSpeed = min(abs(accelerationCurve), abs(decelerationCurve));
	const float constrainedSpeed = constrain(computedSpeed, minSpeed, configuration->maxSpeed);
	currentVelocity = constrainedSpeed * direction;

	// Position Error
	int32_t currentPositionError = computePositionError(currentPosition);

	// Integral Error
	integralError = integralError + currentPositionError*deltaTime;
	
	// Derivative Error
	float derivativeError = (currentPositionError-positionError)/(deltaTime);

	// Store previous error for future calculation
	positionError = currentPositionError;

	// Compute PWM / DIR
	float pwm = PID.DCMOTOR_kp*currentPositionError + PID.DCMOTOR_kd*derivativeError + PID.DCMOTOR_ki*integralError;
	bool dir = 0;
	if (pwm < 0)
		dir = 1;
	if (abs(currentPositionError) <= static_cast<int>(ROTATOR_DEFAULT_DEADZONE)){ // If positionError is <= deadband, stop
		hardStop();
	} else {
		pwm = constrain(abs(pwm), MOTOR_MIN_PWM, 255);
		_rotator->run(dir,static_cast<int>(pwm));
	}
	}

int32_t DCMotor::computePositionError(int32_t currentPosition)
	{
	int32_t positionError = targetPosition - currentPosition;
	int32_t max_position = limitOfTravel();
	// Calculate the actual position error based on the shortest path distance to the target
	int32_t wraparoundError = ((positionError + (max_position/2)) % max_position) - (max_position/2);
	// Return absolute
	//return configuration->wraparound = true ? wraparoundError : positionError;
	return positionError;
	}