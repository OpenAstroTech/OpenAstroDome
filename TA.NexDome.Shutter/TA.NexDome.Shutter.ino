#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ArduinoSTL.h>
#include <sstream>
#include <AdvancedStepper.h>
#include <XBeeApi.h>
#include "NexDome.h"
#include "XBeeStateMachine.h"
#include "XBeeStartupState.h"
#include "CommandProcessor.h"
#include "PersistentSettings.h"
#include "LimitSwitch.h"

Timer periodicTasks;
auto stepGenerator = CounterTimer1StepGenerator();
auto settings = PersistentSettings::Load();
auto stepper = MicrosteppingMotor(MOTOR_STEP_PIN, MOTOR_ENABLE_PIN, MOTOR_DIRECTION_PIN, stepGenerator, settings.motor);
auto commandProcessor = CommandProcessor(stepper, settings);
auto& xbeeSerial = Serial1;
auto& host = Serial;
std::string hostReceiveBuffer;
std::vector<byte> xbeeApiRxBuffer;
void HandleFrameReceived(FrameType type, const std::vector<byte>& payload);	// forward reference

auto xbee = XBeeApi(xbeeSerial, xbeeApiRxBuffer, ReceiveHandler(HandleFrameReceived));
auto machine = XBeeStateMachine(xbeeSerial, xbee);
auto limitSwitches = LimitSwitch(&stepper, OPEN_LIMIT_SWITCH_PIN, CLOSED_LIMIT_SWITCH_PIN);

void HandleFrameReceived(FrameType type, const std::vector<byte>& payload)
{
	machine.onXbeeFrameReceived(type, payload);
}


Response DispatchCommand(const std::string& buffer)
{
	auto charCount = buffer.length();
	if (charCount < 2)
		return Response::Error();
	Command command;
	command.RawCommand = buffer;
	command.StepPosition = 0;
	command.Verb.push_back(buffer[1]);
	if (charCount > 2)
		command.Verb.push_back(buffer[2]);
	// If there is no device address then use '0', the default device.
	if (charCount < 4)
	{
		command.TargetDevice = '0';
		return commandProcessor.HandleCommand(command);
	}
	// Use the device address from the command
	command.TargetDevice = buffer[3];
	// If the parameter was present, then parse it as an integer; otherwise use 0.
	if (charCount > 5 && buffer[4] == ',')
	{
		auto position = buffer.substr(5);
		auto wholeSteps = std::strtoul(position.begin(), NULL, 10);
		command.StepPosition = wholeSteps;
	}
	auto response = commandProcessor.HandleCommand(command);
	return response;
}


void HandleSerialCommunications()
{
	if (host.available() <= 0)
		return;	// No data available.
	auto rx = host.read();
	if (rx < 0)
		return;	// No data available.
	char rxChar = (char)rx;
	switch (rxChar)
	{
	case '\n':	// newline - dispatch the command
	case '\r':	// carriage return - dispatch the command
		if (hostReceiveBuffer.length() > 1)
		{
			hostReceiveBuffer.push_back(rxChar);	// include the EOL in the receive buffer.
			auto response = DispatchCommand(hostReceiveBuffer);
			std::cout << response;	// send a fully formatted response, or nothing if there is no response.
			hostReceiveBuffer.clear();
		}
		break;
	case '@':	// Start of new command
		hostReceiveBuffer.clear();
	default:
		if (hostReceiveBuffer.length() < SERIAL_RX_BUFFER_SIZE)
		{
			hostReceiveBuffer.push_back(rxChar);
		}
		break;
	}
}

// the setup function runs once when you press reset or power the board
void setup() {
	stepper.ReleaseMotor();
	hostReceiveBuffer.reserve(SERIAL_RX_BUFFER_SIZE);
	xbeeApiRxBuffer.reserve(API_MAX_FRAME_LENGTH);
	host.begin(115200);
	xbeeSerial.begin(9600);
	while (!Serial) ;	// Wait for Leonardo software USB stack to become active
	delay(1000);		// Let the USB/serial stack warm up.
	periodicTasks.SetDuration(1000);
	interrupts();
	std::cout << "Init" << std::endl;
	machine.ChangeState(new XBeeStartupState(machine));
	limitSwitches.init();	// attaches interrupt vectors
}

// the loop function runs over and over again until power down or reset
void loop() {
	static std::ostringstream converter;
	stepper.Loop();
	HandleSerialCommunications();
	machine.Loop();
	if (periodicTasks.Expired())
	{
		periodicTasks.SetDuration(250);
		if (stepper.CurrentVelocity() != 0.0)
		{
			converter.clear();
			converter.str("");
			converter << "S" << stepper.CurrentPosition();
			std::cout << converter.str() << std::endl;
			machine.SendToRemoteXbee(converter.str());
		}
	}
}
