#include "FlywheelEncoder.h"



FlywheelEncoder::FlywheelEncoder(DigitalInput &lightSensor):
	flywheelLightSensor(lightSensor)
{
	flywheelCounter = 0;
	flywheelStopwatch.Reset();
	flywheelStopwatch.Start();
}

double FlywheelEncoder::PeriodCounter(){
	rotationsPerPeriod = static_cast<double>(flywheelCounter) / flywheelStopwatch.Get();
	flywheelCounter = 0;
	flywheelStopwatch.Reset();
	flywheelStopwatch.Start();
	return rotationsPerPeriod;
}

double FlywheelEncoder::GetRate(){
	return rotationsPerPeriod;
}
void FlywheelEncoder::FlywheelCounter(){
	bool currentLightValue = flywheelLightSensor.Get();
	static bool previousLightValue = false;
	if (currentLightValue == true && previousLightValue == false) {
		flywheelCounter++;
	}
	previousLightValue = currentLightValue;
}
double FlywheelEncoder::PIDGet(){
	// Change periods(0.5s) to rpm(1 min)
	return rotationsPerPeriod;
}
