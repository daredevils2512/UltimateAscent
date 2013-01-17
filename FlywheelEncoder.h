#include "WPILib.h"

DigitalInput *flywheelLightSensor;
Timer flywheelStopwatch;
PIDController *flywheelSpeed;


/* This class counts how many times the flywheel has turned during a period
 * custom made for one-click-encoder
 */

class FlywheelEncoder:PIDSource {
	
private:	
	
	int rate;
	static const double period;
	int flywheelCounter;
	int rotationsPerPeriod;
	
public:
	//Default Constructer, initializzes thinz to zero
	FlywheelEncoder () {
		rate = 0;
		flywheelCounter = 0;
		flywheelStopwatch.Reset();
		flywheelStopwatch.Start();
	}
	
	// Counts number of rotations per period
	int periodCounter () {
		if (flywheelStopwatch.Get() >= period) {
			rotationsPerPeriod = flywheelCounter;
			flywheelCounter = 0;
			flywheelStopwatch.Reset();
			flywheelStopwatch.Start();
		}
		return rotationsPerPeriod;
	}
	
	double getRate () {
		// Counts rotations with rising edge detector
		bool currentLightValue = flywheelLightSensor->Get();
		static bool previousLightValue = false;
		if (currentLightValue == true && previousLightValue == false) {
			flywheelCounter++;
		}
		previousLightValue = currentLightValue;
		// Change periods(0.5s) to rpm(1 min)
		rate = periodCounter () * 120;
		return rate;
	}
	
	virtual double PIDGet(){
		return getRate();
	}
};

const double FlywheelEncoder::period = 0.5;
