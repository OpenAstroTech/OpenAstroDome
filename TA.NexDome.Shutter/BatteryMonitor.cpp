﻿
#include <sstream>
#include "BatteryMonitor.h"
#include "Response.h"
#include "CommandProcessor.h"
extern Response DispatchCommand(const std::string &buffer);

BatteryMonitor::BatteryMonitor(XBeeStateMachine &machine, uint8_t analogPin, BatteryMonitorSettings &settings)
	: machine(machine), analogPin(analogPin), settings(settings),
	  movingAverageVoltage(settings.sampleWindow)
{
	sampleTimer.Stop();
	notificationTimer.Stop();
}

void BatteryMonitor::initialize(unsigned long initialDelay)
{
	sampleTimer.SetDuration(initialDelay);
	notificationTimer.SetDuration(settings.notifyInterval);
	movingAverageVoltage.init(65535);
}

void BatteryMonitor::loop()
{
	if (!sampleTimer.Expired())
		return;
	sampleTimer.SetDuration(settings.sampleInterval);
	movingAverageVoltage.addSample(static_cast<const int>(analogRead(analogPin)));
	checkThresholdAndSendNotification();
}

bool BatteryMonitor::lowVolts()
{
	return movingAverageVoltage.average < settings.threshold;
}

void BatteryMonitor::checkThresholdAndSendNotification()
{
	if (!notificationTimer.Expired())
		return;
	notificationTimer.SetDuration(settings.notifyInterval);
	std::ostringstream message;
	message << Response::header << "BV" << movingAverageVoltage.average << Response::terminator;
	machine.SendToRemoteXbee(message.str());
#ifdef SHUTTER_LOCAL_OUTPUT
	std::cout << message.str() << std::endl;
#endif
	if (lowVolts())
	{
		const std::string closeShutter = "@CLS";
		const std::string lowVoltsMessage = "Volts";
		machine.SendToRemoteXbee(lowVoltsMessage);
#ifdef SHUTTER_LOCAL_OUTPUT
		std::cout << lowVoltsMessage << std::endl;
#endif
		::DispatchCommand(closeShutter);
	}
}
