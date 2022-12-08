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

void onMotorLeftStopped(); // Forward reference
void onMotorRightStopped(); // Forward reference

Timer periodicTasks;
auto stepGenerator = CounterTimer1StepGenerator();
auto settings = PersistentSettings::Load();
#if SHUTTER_MOTOR_TYPE == STEPPER_MOTOR
	auto stepperLeft = MicrosteppingMotor(LEFT_MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, LEFT_MOTOR_DIRECTION_PIN, stepGenerator, settings.motor);
	auto stepperRight = MicrosteppingMotor(RIGHT_MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, RIGHT_MOTOR_DIRECTION_PIN, stepGenerator, settings.motor);
#elif SHUTTER_MOTOR_TYPE == DC_MOTOR
	auto stepper = DCMotor(MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, MOTOR_DIRECTION_PIN, settings.motor);
#endif
auto limitSwitchesLeft = LimitSwitch(&stepperLeft, LEFT_LIMIT_SWITCH_PIN);
auto limitSwitchesRight = LimitSwitch(&stepperRight, RIGHT_LIMIT_SWITCH_PIN);

auto &xbeeSerial = XBEE_SERIAL;
HardwareSerial host(Serial);
std::string hostReceiveBuffer;
std::vector<byte> xbeeApiRxBuffer;
void HandleFrameReceived(FrameType type, const std::vector<byte> &payload); // forward reference
auto xbee = XBeeApi(xbeeSerial, xbeeApiRxBuffer, ReceiveHandler(HandleFrameReceived));
auto machine = XBeeStateMachine(xbeeSerial, xbee);
auto batteryMonitor = BatteryMonitor(machine, BATTERY_SENSOR_PIN, settings.batteryMonitor);
auto commandProcessor = CommandProcessor(stepperLeft, stepperRight, settings, machine, limitSwitchesLeft, limitSwitchesRight, batteryMonitor);

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
	//std::cout << openButtonPressed;
	const bool openButtonChanged = openButtonPressed != openButtonLastState;
	if (openButtonChanged && openButtonPressed)
	{
		// std::cout << "OPEN";
		commandProcessor.sendOpenNotification();
		stepperLeft.moveToPosition(MaxStepPosition);
		stepperRight.moveToPosition(MaxStepPosition);
	}
	if (openButtonChanged && !openButtonPressed)
	{
		stepperLeft.SoftStop();
		stepperRight.SoftStop();
	}
	openButtonLastState = openButtonPressed;
	const bool closedButtonPressed = digitalRead(CLOSE_BUTTON_PIN) == 0;
	//std::cout << closedButtonPressed;
	const bool closedButtonChanged = closedButtonPressed != closeButtonLastState;
	if (closedButtonChanged && closedButtonPressed)
	{
		// std::cout << "CLOSE";
		commandProcessor.sendCloseNotification();
		stepperLeft.moveToPosition(MinStepPosition);
		stepperRight.moveToPosition(MinStepPosition);
	}
	if (closedButtonChanged && !closedButtonPressed)
	{
		stepperLeft.SoftStop();
		stepperRight.SoftStop();
	}
	closeButtonLastState = closedButtonPressed;
}

void DispatchCommand(const Command &command)
{
	commandProcessor.HandleCommand(command);
}

void HandleSerialCommunications()
{
	while (Serial.available() > 0)
	{
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
}

// the setup function runs once when you press reset or power the board
void setup()
{
	stepperLeft.releaseMotor();
	stepperRight.releaseMotor();
	stepperLeft.registerStopHandler(onMotorLeftStopped);
	stepperRight.registerStopHandler(onMotorRightStopped);

	pinMode(CLOCKWISE_BUTTON_PIN, INPUT_PULLUP);
	pinMode(COUNTERCLOCKWISE_BUTTON_PIN, INPUT_PULLUP);

	hostReceiveBuffer.reserve(HOST_SERIAL_RX_BUFFER_SIZE);
	xbeeApiRxBuffer.reserve(API_MAX_FRAME_LENGTH);

	host.begin(115200);

	// Connect cin and cout to our SafeSerial instance
	ArduinoSTL_Serial.connect(Serial);
	xbeeSerial.begin(9600);

	periodicTasks.SetDuration(1000);
	interrupts();
	machine.ChangeState(new XBeeStartupState(machine));
	machine.ChangeState(new XBeeOnlineState(machine));
	limitSwitchesLeft.init(); // attaches interrupt vectors
	limitSwitchesRight.init(); // attaches interrupt vectors
#if !DEBUG_CONSERVE_FLASH
	batteryMonitor.initialize(10000);
#endif
}

// the loop function runs over and over again until power down or reset
void loop()
{
	static std::ostringstream converter;
	stepperLeft.loop();
	stepperRight.loop();
	HandleSerialCommunications();
	machine.Loop();

#if !DEBUG_CONSERVE_FLASH
	batteryMonitor.loop();
#endif
	if (periodicTasks.Expired())
	{
		periodicTasks.SetDuration(250);
		ProcessManualControls();
		if (stepperLeft.isMoving())
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
void onMotorLeftStopped()
{
	limitSwitchesLeft.onMotorStopped();
	commandProcessor.sendStatus();
}

void onMotorRightStopped()
{
	limitSwitchesRight.onMotorStopped();
	commandProcessor.sendStatus();
}

