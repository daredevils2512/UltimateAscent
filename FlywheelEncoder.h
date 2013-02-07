#include <WPILib.h>


/* This class counts how many times the flywheel has turned during a period
 * custom made for one-click-encoder
 */

class FlywheelEncoder:public PIDSource {
	
private:	

	DigitalInput &flywheelLightSensor;
	Timer flywheelStopwatch;
	
	static const double period;
	int flywheelCounter;
	int rotationsPerPeriod;
	
public:
	//Default Constructer, initializzes thinz to zero
	FlywheelEncoder (DigitalInput &lightSensor);
	
	//Destructor
	virtual ~FlywheelEncoder(){}
	
	// Counts number of rotations per period
	int PeriodCounter ();
	
	// Counts rotations with rising edge detector
	double GetRate ();
	
	//Inherited function for PID source recognition
	virtual double PIDGet();
};
