#include "WPILib.h"

DigitalInput flywheelLightSensor;
Timer flywheelStopwatch;
PIDController flywheelSpeed;
static const UINT32 FLYWHEEL_ON = 8;
static const UINT32 FLYWHEEL_OFF = 9;

/* This class counts how many times the flywheel has turned during a period
 * custom made for one-click-encoder
 */

class FlywheelEncoder {
	
private:	
	
	int rate;
	const double period = 0.5;
	int flywheelCounter;
	
public:
	
	FlywheelEncoder () { //Default Constructer, initializzes thinz to zero
		rate = 0;
		flywheelCounter = 0;
		flywheelStopwatch.Reset();
		flywheelStopwatch.Start();
	}
	
	int periodCounter () { // Counts number of rotations per period
		if (flywheelStopwatch.Get() >= period) {
			int rotationsPerPeriod = flywheelCounter;
			flywheelCounter = 0;
			flywheelStopwatch.Reset();
			flywheelStopwatch.Start();
		}
		return rotationsPerPeriod;
	}
	
	double getRate () { // Sets rate to rpm
		if (flywheelLightSensor.Get() == true) { // Counts rotations
			flywheelCounter++;
		}
		rate = periodCounter () * 120; // change to rpm (60/.5)
		return rate;
	}
};
