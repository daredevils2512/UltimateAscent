#include <WPILib.h>
#include <al.hpp>
#include "FlywheelEncoder.h"
#include "FlywheelMotor.h"
#pragma once

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
	static const UINT32 FRONT_LEFT_MOTOR_PWM = 6;
	// Front Right Motor
	static const UINT8 FRONT_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FRONT_RIGHT_MOTOR_PWM = 4;
	// Rear Left Motor
	static const UINT8 REAR_LEFT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 REAR_LEFT_MOTOR_PWM = 7;
	// Rear Right Motor
	static const UINT8 REAR_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 REAR_RIGHT_MOTOR_PWM = 2;
	// Solenoid 1
	static const UINT8 SOLENOID1_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SOLENOID1_PWM = 3;
	// Solenoid 2
	static const UINT8 SOLENOID2_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SOLENOID2_PWM = 4;
	// Solenoid 3
	static const UINT8 SOLENOID3_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SOLENOID3_PWM = 7;
	// Solenoid 4
	static const UINT8 SOLENOID4_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SOLENOID4_PWM = 8;
	// Scoop Solenoid 1
	static const UINT8 SCOOP_SOLENOID1_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SCOOP_SOLENOID1_PWM = 5;
	// Scoop Solenoid 2
	static const UINT8 SCOOP_SOLENOID2_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 SCOOP_SOLENOID2_PWM = 6;
	// Flywheel Light Sensor
	static const UINT8 FLWYHEEL_LIGHT_SENSOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FLYWHEEL_LIGHT_SENSOR_PWM = 5;
	// Flywheel Motor
	static const UINT8 FLYWHEEL_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FLYWHEEL_MOTOR_PWM = 3;	
	// Brush Motor
	static const UINT8 BRUSH_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 BRUSH_MOTOR_PWM = 1;
	// Elevator Motor
	static const UINT8 ELEVATOR_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 ELEVATOR_MOTOR_PWM = 5;
	// Frisbee Light Sensor
	static const UINT8 FRISBEE_LIGHT_SENSOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 FRISBEE_LIGHT_SENSOR_PWM = 9;
	// Left Motor Encoder
	static const UINT32 LEFT_MOTOR_ENCODER_PWM_A = 3;
	static const UINT32 LEFT_MOTOR_ENCODER_PWM_B = 4;
	// Right Motor Encoder
	static const UINT32 RIGHT_MOTOR_ENCODER_PWM_A = 1;
	static const UINT32 RIGHT_MOTOR_ENCODER_PWM_B = 2;
	//Shooter Angle Motor
	static const UINT8 SHOOTER_ANGLE_MOTOR_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 SHOOTER_ANGLE_MOTOR_PWM = 2;
	// Launcher In
	static const UINT8 LAUNCHER_IN_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 LAUNCHER_IN_PWM = 1;
	// Launcher Out
	static const UINT8 LAUNCHER_OUT_SIDECAR = SOLENOID_SIDECAR_1;
	static const UINT32 LAUNCHER_OUT_PWM = 2;
	// Potentiometer
	static const UINT8 POTENTIOMETER_SIDECAR = ANALOG_SIDECAR_1;
	static const UINT32 POTENTIOMETER_PWM = 1;
	// Compressor
	static const UINT8 COMPRESSOR_RELAY_SIDECAR = DIGITAL_SIDECAR_1;
	static const UINT32 COMPRESSOR_RELAY_PWM = 1;
	static const UINT8 COMPRESSOR_SWITCH_SIDECAR = DIGITAL_SIDECAR_2;
	static const UINT32 COMPRESSOR_SWITCH_PWM = 6;
	
	//Joystick 1 Buttons
	static const int GRIPPIES_DOWN_BUTTON = 2;
	// Lower Elevator up button
	static const int SCOOP_BUTTON = 7;
	// Lower Elevator down button
	static const int SCOOP_REVERSE_BUTTON = 8;
	static const int SCOOP_UP_BUTTON = 10;
	static const int SCOOP_DOWN_BUTTON = 9;
	
	//Joystick 2 Buttons
	static const UINT32 FIRE_BUTTON = 1;
	static const int FLYWHEEL_ON_BUTTON = 9;
	static const int FLYWHEEL_OFF_BUTTON = 6;
	static const int ANGLE_UP_BUTTON = 11;
	static const int ANGLE_DOWN_BUTTON = 10;
	
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
	Solenoid solenoid3;
	Solenoid solenoid4;
	Solenoid scoopSolenoid1;
	Solenoid scoopSolenoid2;
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
	
	void Shoot();	
	
	void Drive();
	
	void Scoop();
	
	void SetLauncherOut();
	
	void SetLauncherIn();
	
	void SetLauncherFalse();
	
	float ShooterAngle(float pot);
public:
	UltimateAscent(void);

	void Autonomous();

	void OperatorControl();
	
	void Test();
	
};
