#include <Arduino.h>

#include <ArduinoSTL.h>
#include <sstream>
// #include <SafeSerial.h>
#include <AdvancedStepper.h>
#include "DCMotor.h"
#include <XBeeApi.h>
#include "OpenAstroDome.h"
#include "XBeeStatemachine.h"
#include "XBeeStartupState.h"
#include "XBeeWaitForCommandModeState.h"
#include "XBeeOnlineState.h"
#include "CommandProcessor.h"
#include "PersistentSettings.h"
#include "LimitSwitch.h"
#include "BatteryMonitor.h"
#include "Shutters.h"

// Display
#include "sdd1306.h"

void onRightMotorStopped(); // Forward reference
void onLeftMotorStopped(); // Forward reference

// Display
SDD1306* display = new SDD1306();

Timer periodicTasks;
//auto rightStepGenerator = CounterTimer1StepGenerator();
//auto leftStepGenerator = CounterTimer1StepGenerator();

// auto stepGenerator = CounterTimer1StepGenerator();
PersistentSettings settings = PersistentSettings::Load();
#if SHUTTER_MOTOR_TYPE == STEPPER_MOTOR
	Shutters* shutters = new Shutters(settings.motor);
	//MicrosteppingMotor* rightStepper = new MicrosteppingMotor(RIGHT_MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, RIGHT_MOTOR_DIRECTION_PIN, rightStepGenerator, settings.motor, false);
	//MicrosteppingMotor* leftStepper = new MicrosteppingMotor(LEFT_MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, LEFT_MOTOR_DIRECTION_PIN, leftStepGenerator, settings.motor, true);
#elif SHUTTER_MOTOR_TYPE == DC_MOTOR
	auto stepper = DCMotor(MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, MOTOR_DIRECTION_PIN, settings.motor);
#endif
auto limitSwitchesRight = LimitSwitchRight(shutters, RIGHT_OPEN_LIMIT_SWITCH_PIN, RIGHT_CLOSED_SWITCH_PIN);
auto limitSwitchesLeft = LimitSwitchLeft(shutters, LEFT_OPEN_LIMIT_SWITCH_PIN, LEFT_CLOSED_LIMIT_SWITCH_PIN);
auto &xbeeSerial = XBEE_SERIAL; // Original
//HardwareSerial host(Serial);
std::string hostReceiveBuffer;
std::vector<byte> xbeeApiRxBuffer;
void HandleFrameReceived(FrameType type, const std::vector<byte> &payload); // forward reference
auto xbee = XBeeApi(xbeeSerial, xbeeApiRxBuffer, ReceiveHandler(HandleFrameReceived));
auto machine = XBeeStateMachine(xbeeSerial, xbee);
auto batteryMonitor = BatteryMonitor(machine, BATTERY_SENSOR_PIN, settings.batteryMonitor);
auto commandProcessor = CommandProcessor(shutters, settings, machine, limitSwitchesRight, limitSwitchesLeft, batteryMonitor, display);

// cin and cout for ArduinoSTL

std::ohserialstream cout(Serial);
std::ihserialstream cin(Serial);



void HandleFrameReceived(FrameType type, const std::vector<byte> &payload)
{
	machine.onXbeeFrameReceived(type, payload);
}

void ProcessManualControls()
{
	static bool openButtonLastState = false;
	static bool closeButtonLastState = false;
	const bool openButtonPressed = digitalRead(OPEN_BUTTON_PIN) == 0;
	const bool openButtonChanged = openButtonPressed != openButtonLastState;
	if (openButtonChanged && openButtonPressed)
	{
		commandProcessor.sendOpenNotification();
		shutters->rightStepper->moveTo(MaxStepPosition);
		shutters->leftStepper->moveTo(MaxStepPosition);
	}
	if (openButtonChanged && !openButtonPressed)
	{
		//rightStepper->SoftStop();
		shutters->hardStop();
	}
	openButtonLastState = openButtonPressed;
	const bool closedButtonPressed = digitalRead(CLOSE_BUTTON_PIN) == 0;
	const bool closedButtonChanged = closedButtonPressed != closeButtonLastState;
	if (closedButtonChanged && closedButtonPressed)
	{
		commandProcessor.sendCloseNotification();
		// rightStepper->moveToPosition(MinStepPosition);
		shutters->rightStepper->moveTo(MinStepPosition);
		shutters->leftStepper->moveTo(MinStepPosition);
	}
	if (closedButtonChanged && !closedButtonPressed)
	{
		shutters->hardStop();
		//rightStepper->SoftStop();
	}
	closeButtonLastState = closedButtonPressed;

}

void DispatchCommand(const Command &command)
{
	commandProcessor.HandleCommand(command);
}

void HandleSerialCommunications()
	{
	if (!Serial || Serial.available() <= 0)
		return; // No data available.
	const auto rx = Serial.read();
	if (rx < 0)
		return; // No data available.
	const char rxChar = char(rx);
	switch (rxChar)
		{
		case '\n': // newline - dispatch the command
		case '\r': // carriage return - dispatch the command
			if (hostReceiveBuffer.length() > 1)
				{
                const auto command = Command(hostReceiveBuffer);
				DispatchCommand(command);
				if (ResponseBuilder::available())
				    std::cout << ResponseBuilder::Message << std::endl; // send response, if there is one.
				hostReceiveBuffer.clear();
				}
			break;
		case '@': // Start of new command
			hostReceiveBuffer.clear();
		default:
			if (hostReceiveBuffer.length() < HOST_SERIAL_RX_BUFFER_SIZE)
				{
				hostReceiveBuffer.push_back(rxChar);
				}
			break;
		}
	}

// the setup function runs once when you press reset or power the board
void setup()
{
	display->setWelcome("Shutter");
	display->init();
	display->setTitel("-- Shutter Status --");

	// rightStepper->releaseMotor();
	// leftStepper->releaseMotor();
	// shutters->releaseMotor();
	shutters->init();

	shutters->registerRightStopHandler(onRightMotorStopped);
	shutters->registerLeftStopHandler(onLeftMotorStopped);
	pinMode(CLOCKWISE_BUTTON_PIN, INPUT_PULLUP);
	pinMode(COUNTERCLOCKWISE_BUTTON_PIN, INPUT_PULLUP);
	hostReceiveBuffer.reserve(HOST_SERIAL_RX_BUFFER_SIZE);
	xbeeApiRxBuffer.reserve(API_MAX_FRAME_LENGTH);
	Serial.begin(115200);

	// Connect cin and cout to our SafeSerial instance
	ArduinoSTL_Serial.connect(Serial);
	xbeeSerial.begin(9600);

	periodicTasks.SetDuration(1000);
	interrupts();
	machine.ChangeState(new XBeeStartupState(machine));
	//machine.ChangeState(new XBeeOnlineState(machine));
	limitSwitchesRight.init(); // attaches interrupt vectors
	limitSwitchesLeft.init(); // attaches interrupt vectors
#if !DEBUG_CONSERVE_FLASH
	batteryMonitor.initialize(10000);
#endif

	//settings.motor.directionReversed = false;
	settings.motor.directionReversed = true;
	//settings.motor.maxPosition = 20000L;
	//settings.motor.maxSpeed = 200;
	//settings.motor.rampTimeMilliseconds = 1500;

}

// the loop function runs over and over again until power down or reset
void loop()
{
	static std::ostringstream converter;
	//rightStepper->loop();
	//leftStepper->loop();

	shutters->loop();
	
	HandleSerialCommunications();
	machine.Loop();
	
#if !DEBUG_CONSERVE_FLASH
	batteryMonitor.loop();
#endif
	if (periodicTasks.Expired())
	{
		periodicTasks.SetDuration(250);
		ProcessManualControls();
		
		if (!shutters->rightStepper->isRunning())
		{
			display->setMessage(machine.GetStateName());
			display->displayXBeeStatus();
		}
		

		if (shutters->rightStepper->isRunning())
		{
			const auto wholeSteps = commandProcessor.getPositionInWholeSteps();
			converter.clear();
			converter.str("");
			converter << "S" << wholeSteps;
#ifdef SHUTTER_LOCAL_OUTPUT
			std::cout << "S" << std::dec << wholeSteps << std::endl;
#endif
			machine.SendToRemoteXbee(converter.str());
		}
	}
	
}

// Handle the motor stop event from the stepper driver.
void onRightMotorStopped()
{
	limitSwitchesRight.onMotorStopped();
	commandProcessor.sendStatus();
}

void onLeftMotorStopped()
{
	limitSwitchesLeft.onMotorStopped();
}