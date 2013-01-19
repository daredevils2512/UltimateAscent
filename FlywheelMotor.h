#include"WPILib.h"

class FlywheelMotor:public PIDOutput {
	
private:

public:
	FlywheelMotor();
	
	virtual ~FlywheelMotor();
	
	virtual void PIDWrite(float output);
};
