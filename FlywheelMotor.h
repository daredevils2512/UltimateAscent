#include <WPILib.h>
#pragma once

class FlywheelMotor:public PIDOutput {
	
private:
Talon &flywheelMotor;
	
public:
	FlywheelMotor(Talon &motor);
	
	virtual ~FlywheelMotor(){}
	
	virtual void PIDWrite(float output);
};
