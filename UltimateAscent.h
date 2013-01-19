#include "WPILib.h"
#include "FlywheelEncoder.h"
#include "FlywheelMotor.h"

class RobotDemo : public SimpleRobot
{
	//SideCar Constants
	static const UINT8 DIGITAL_SIDECAR_1 = 1;
	static const UINT8 DIGITAL_SIDECAR_2 = 2;
	static const UINT8 ANALOG_SIDECAR_1 = 1;
	static const UINT8 SOLENOID_SIDECAR_1 = 1;
	
	//Sidecar list
	static const UINT8 FRONT_LEFT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT8 FRONT_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT8 REAR_LEFT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT8 REAR_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT8 SOLENOID1_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT8 SOLENOID2_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT8 SCOOP_SOLENOID_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT8 FLWYHEEL_LIGHT_SENSOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT8 FLYWHEEL_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	
	//PWM list
	static const UINT32 FRONT_LEFT_MOTOR_PWM = 1;
	static const UINT32 FRONT_RIGHT_MOTOR_PWM = 2;
	static const UINT32 REAR_LEFT_MOTOR_PWM = 3;
	static const UINT32 REAR_RIGHT_MOTOR_PWM = 4;
	static const UINT32 SOLENOID1_PWM = 1;
	static const UINT32 SOLENOID2_PWM = 2;
	static const UINT32 SCOOP_SOLENOID_PWM = 3;
	static const UINT32 FLYWHEEL_LIGHT_SENSOR_PWM = 5;
	static const UINT32 FLYWHEEL_MOTOR_PWM = 6;
	
	//Buttons
	
	static const int GRIPPIES_DOWN_BUTTON = 2;
	static const int FLYWHEEL_ON_BUTTON = 9;
	static const int FLYWHEEL_OFF_BUTTON = 6;
	static const int SCOOP_BUTTON = 7;
	
	//VARIABLES

	
	Timer timer;
	Talon frontLeftMotor;
	Talon frontRightMotor;
	Talon rearLeftMotor;
	Talon rearRightMotor;
	Talon flywheelMotor;
	Solenoid solenoid1;
	Solenoid solenoid2;
	Solenoid scoopSolenoid;
	DigitalInput flywheelLightSensor;
	FlywheelEncoder flywheelEncoder;
	FlywheelMotor pidOutput;
	PIDController flywheelSpeed;
	RobotDrive myRobot; // robot drive system
	Joystick stick1; // only joystick
	Joystick stick2;
	
	float ConvertAxis(float input);
	
public:
	RobotDemo(void);

	void Autonomous();

	void OperatorControl();
	
	void Test();
	
	void Shoot();	
	
	void Drive();
	
	void Scoop();
};
