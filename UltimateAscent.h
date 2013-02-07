#include "WPILib.h"
#include <al.hpp>
#include "FlywheelEncoder.h"
#include "FlywheelMotor.h"

class UltimateAscent : public SimpleRobot
{
	//SideCar Constants
	static const UINT8 DIGITAL_SIDECAR_1 = 1;
	static const UINT8 DIGITAL_SIDECAR_2 = 2;
	static const UINT8 ANALOG_SIDECAR_1 = 1;
	static const UINT8 SOLENOID_SIDECAR_1 = 1;
	
	//Sidecar list
	// Front Left Motor
	static const UINT8 FRONT_LEFT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FRONT_LEFT_MOTOR_PWM = 1;
	// Front Right Motor
	static const UINT8 FRONT_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FRONT_RIGHT_MOTOR_PWM = 2;
	// Rear Left Motor
	static const UINT8 REAR_LEFT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 REAR_LEFT_MOTOR_PWM = 3;
	// Rear Right Motor
	static const UINT8 REAR_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 REAR_RIGHT_MOTOR_PWM = 4;
	// Solenoid 1
	static const UINT8 SOLENOID1_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SOLENOID1_PWM = 1;
	// Solenoid 2
	static const UINT8 SOLENOID2_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SOLENOID2_PWM = 2;
	// Scoop Solenoid
	static const UINT8 SCOOP_SOLENOID_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SCOOP_SOLENOID_PWM = 3;
	// Flywheel Light Sensor
	static const UINT8 FLWYHEEL_LIGHT_SENSOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FLYWHEEL_LIGHT_SENSOR_PWM = 5;
	// Flywheel Motor
	static const UINT8 FLYWHEEL_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FLYWHEEL_MOTOR_PWM = 6;	
	// Brush Motor
	static const UINT8 BRUSH_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 BRUSH_MOTOR_PWM = 7;
	// Elevator Motor
	static const UINT8 ELEVATOR_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 ELEVATOR_MOTOR_PWM = 8;
	// Frisbee Light Sensor
	static const UINT8 FRISBEE_LIGHT_SENSOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FRISBEE_LIGHT_SENSOR_PWM = 9;
	// Left Motor Encoder
	static const UINT32 LEFT_MOTOR_ENCODER_PWM_A = 1;
	static const UINT32 LEFT_MOTOR_ENCODER_PWM_B = 2;
	// Right Motor Encoder
	static const UINT32 RIGHT_MOTOR_ENCODER_PWM_A = 3;
	static const UINT32 RIGHT_MOTOR_ENCODER_PWM_B = 4;
	//Shooter Angle Motor
	static const UINT8 SHOOTER_ANGLE_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 SHOOTER_ANGLE_MOTOR_PWM = 1;
	// Launcher In
	static const UINT8 LAUNCHER_IN_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 LAUNCHER_IN_PWM = 3;
	// Launcher Out
	static const UINT8 LAUNCHER_OUT_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 LAUNCHER_OUT_PWM = 2;
	// Potentiometer
	static const UINT8 POTENTIOMETER_SIDECAR = ANALOG_SIDECAR_1;
	static const UINT32 POTENTIOMETER_PWM = 1;
	// Compressor
	static const UINT8 COMPRESSOR_RELAY_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 COMPRESSOR_RELAY_PWM = 2;
	static const UINT8 COMPRESSOR_SWITCH_SIDECAR = DIGITAL_SIDECAR_2;
	static const UINT32 COMPRESSOR_SWITCH_PWM = 5;
	
	//Buttons
	static const UINT32 FIRE_BUTTON = 1;
	static const int GRIPPIES_DOWN_BUTTON = 2;
	static const int FLYWHEEL_ON_BUTTON = 9;
	static const int FLYWHEEL_OFF_BUTTON = 6;
	static const int SCOOP_BUTTON = 7;
	
	//VARIABLES
	int frisbeeCount;
	
	al::logger log;
	
	Timer timer;
	Talon frontLeftMotor;
	Talon frontRightMotor;
	Talon rearLeftMotor;
	Talon rearRightMotor;
	Talon flywheelMotor;
	Talon brushMotor;
	Talon elevatorMotor;
	Relay shooterAngleMotor;
	Compressor compressor;
	Solenoid solenoid1;
	Solenoid solenoid2;
	Solenoid scoopSolenoid;
	Solenoid launcherIn;
	Solenoid launcherOut;
	DigitalInput flywheelLightSensor;
	DigitalInput frisbeeLightSensor;
	FlywheelEncoder flywheelEncoder;
	Encoder leftMotorEncoder;
	Encoder rightMotorEncoder;
	Timer stopwatch;
	FlywheelMotor pidOutput;
	PIDController flywheelSpeed;
	RobotDrive myRobot; // robot drive system
	Joystick stick1; // only joystick
	Joystick stick2;
	AnalogChannel potentiometer;
	
	
	al::logger CreateLogger();
	float ConvertAxis(float input);
	
public:
	UltimateAscent(void);

	void Autonomous();

	void OperatorControl();
	
	void Test();
	
	void Shoot();	
	
	void Drive();
	
	void Scoop();
};
