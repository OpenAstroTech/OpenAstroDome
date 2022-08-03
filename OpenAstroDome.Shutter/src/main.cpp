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
// UNO SoftwareSerial
#include "SoftwareSerial.h"

void onMotorStopped(); // Forward reference

Timer periodicTasks;
auto stepGenerator = CounterTimer1StepGenerator();
auto settings = PersistentSettings::Load();
#if SHUTTER_MOTOR_TYPE == STEPPER_MOTOR
auto stepper = MicrosteppingMotor(MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, MOTOR_DIRECTION_PIN, stepGenerator, settings.motor);
#elif SHUTTER_MOTOR_TYPE == DC_MOTOR
auto stepper = DCMotor(MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, MOTOR_DIRECTION_PIN, settings.motor);
#endif
auto limitSwitches = LimitSwitch(&stepper, OPEN_LIMIT_SWITCH_PIN, CLOSED_LIMIT_SWITCH_PIN);
// auto &xbeeSerial = Serial1; // Original
auto xbeeSerial = SoftwareSerial(6, 7); // UNO
// HardwareSerial host(Serial);
std::string hostReceiveBuffer;
std::vector<byte> xbeeApiRxBuffer;
void HandleFrameReceived(FrameType type, const std::vector<byte> &payload); // forward reference
auto xbee = XBeeApi(xbeeSerial, xbeeApiRxBuffer, ReceiveHandler(HandleFrameReceived));
auto machine = XBeeStateMachine(xbeeSerial, xbee);
auto batteryMonitor = BatteryMonitor(machine, A0, settings.batteryMonitor);
auto commandProcessor = CommandProcessor(stepper, settings, machine, limitSwitches, batteryMonitor);

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
		stepper.moveToPosition(MaxStepPosition);
	}
	if (openButtonChanged && !openButtonPressed)
	{
		stepper.SoftStop();
	}
	openButtonLastState = openButtonPressed;
	const bool closedButtonPressed = digitalRead(CLOSE_BUTTON_PIN) == 0;
	const bool closedButtonChanged = closedButtonPressed != closeButtonLastState;
	if (closedButtonChanged && closedButtonPressed)
	{
		commandProcessor.sendCloseNotification();
		stepper.moveToPosition(MinStepPosition);
	}
	if (closedButtonChanged && !closedButtonPressed)
	{
		stepper.SoftStop();
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
	stepper.releaseMotor();
	stepper.registerStopHandler(onMotorStopped);
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
	machine.ChangeState(new XBeeOnlineState(machine));
	limitSwitches.init(); // attaches interrupt vectors
#if !DEBUG_CONSERVE_FLASH
	batteryMonitor.initialize(10000);
#endif
}

// the loop function runs over and over again until power down or reset
void loop()
{
	static std::ostringstream converter;
	stepper.loop();
	HandleSerialCommunications();
	machine.Loop();
#if !DEBUG_CONSERVE_FLASH
	batteryMonitor.loop();
#endif
	if (periodicTasks.Expired())
	{
		periodicTasks.SetDuration(250);
		ProcessManualControls();
		if (stepper.isMoving())
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
void onMotorStopped()
{
	limitSwitches.onMotorStopped();
	commandProcessor.sendStatus();
}
