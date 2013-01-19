#include"WPILib.h"
#include"FlywheelMotor.h"


FlywheelMotor::FlywheelMotor(Talon &motor):
	flywheelMotor(&motor)
{
	
}
void FlywheelMotor::PIDWrite(float output){
	flywheelMotor->Set(output);
}
