#include <WPILib.h>
#include "FlywheelEncoder.h"

const double FlywheelEncoder::period = 0.5;

FlywheelEncoder::FlywheelEncoder(DigitalInput &lightSensor):
	flywheelLightSensor(lightSensor)
{
	flywheelCounter = 0;
	flywheelStopwatch.Reset();
	flywheelStopwatch.Start();
}

int FlywheelEncoder::PeriodCounter(){
	if (flywheelStopwatch.Get() >= period) {
		rotationsPerPeriod = flywheelCounter;
		flywheelCounter = 0;
		flywheelStopwatch.Reset();
		flywheelStopwatch.Start();
	}
	return rotationsPerPeriod;
}

double FlywheelEncoder::GetRate(){
	bool currentLightValue = flywheelLightSensor.Get();
	static bool previousLightValue = false;
	if (currentLightValue == true && previousLightValue == false) {
		flywheelCounter++;
	}
	previousLightValue = currentLightValue;
	// Change periods(0.5s) to rpm(1 min)
	return PeriodCounter () * 120;
}

double FlywheelEncoder::PIDGet(){
	return GetRate();
}
