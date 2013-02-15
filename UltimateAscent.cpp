#include <memory>
#include <cmath>
#include "UltimateAscent.h"

using al::smart_ptr;
using al::handler;

UltimateAscent::UltimateAscent(void):
		// these must be initialized in the same order as they are declared in the header file.
		frisbeeCount(0),
		log(CreateLogger()),
		timer(),
		frontLeftMotor(FRONT_LEFT_MOTOR_SIDECAR, FRONT_LEFT_MOTOR_PWM),
		frontRightMotor(FRONT_RIGHT_MOTOR_SIDECAR, FRONT_RIGHT_MOTOR_PWM),
		rearLeftMotor(REAR_LEFT_MOTOR_SIDECAR, REAR_LEFT_MOTOR_PWM),
		rearRightMotor(REAR_RIGHT_MOTOR_SIDECAR, REAR_RIGHT_MOTOR_PWM),
		flywheelMotor(FLYWHEEL_MOTOR_SIDECAR, FLYWHEEL_MOTOR_PWM),
		brushMotor(BRUSH_MOTOR_SIDECAR, BRUSH_MOTOR_PWM),
		elevatorMotor(ELEVATOR_MOTOR_SIDECAR, ELEVATOR_MOTOR_PWM),
		shooterAngleMotor(SHOOTER_ANGLE_MOTOR_SIDECAR, SHOOTER_ANGLE_MOTOR_PWM),
		compressor(COMPRESSOR_SWITCH_SIDECAR, COMPRESSOR_SWITCH_PWM, COMPRESSOR_RELAY_SIDECAR, COMPRESSOR_RELAY_PWM),
		solenoid1(SOLENOID1_SIDECAR, SOLENOID1_PWM),
		solenoid2(SOLENOID2_SIDECAR, SOLENOID2_PWM),
		solenoid3(SOLENOID3_SIDECAR, SOLENOID3_PWM),
		solenoid4(SOLENOID4_SIDECAR, SOLENOID4_PWM),
		scoopSolenoid1(SCOOP_SOLENOID1_SIDECAR, SCOOP_SOLENOID1_PWM),
		scoopSolenoid2(SCOOP_SOLENOID2_SIDECAR, SCOOP_SOLENOID2_PWM),
		launcherIn(LAUNCHER_IN_SIDECAR, LAUNCHER_IN_PWM),
		launcherOut(LAUNCHER_OUT_SIDECAR, LAUNCHER_OUT_PWM),
		flywheelLightSensor(FLWYHEEL_LIGHT_SENSOR_SIDECAR, FLYWHEEL_LIGHT_SENSOR_PWM),
		frisbeeLightSensor(FRISBEE_LIGHT_SENSOR_SIDECAR, FRISBEE_LIGHT_SENSOR_PWM),
		flywheelEncoder(flywheelLightSensor),
		leftMotorEncoder(LEFT_MOTOR_ENCODER_PWM_A, LEFT_MOTOR_ENCODER_PWM_B),
		rightMotorEncoder(RIGHT_MOTOR_ENCODER_PWM_A, RIGHT_MOTOR_ENCODER_PWM_B),
		stopwatch(),
		pidOutput(flywheelMotor),
		flywheelSpeed(1, 0.5, 0, &flywheelEncoder, &pidOutput), // TODO:Tune PID
		myRobot(&frontLeftMotor, &rearLeftMotor, &frontRightMotor, &rearRightMotor),
		stick1(1),
		stick2(2),
		potentiometer(POTENTIOMETER_SIDECAR, POTENTIOMETER_PWM)
	{
		GetWatchdog().SetEnabled(false);
		myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot.SetExpiration(0.1);
		stick1.SetAxisChannel(Joystick::kTwistAxis, 3);
		stick1.SetAxisChannel(Joystick::kThrottleAxis, 4);
//		flywheelSpeed.Enable();
		flywheelSpeed.SetInputRange(0, 3960);
		flywheelSpeed.SetOutputRange(0, 3960);
		compressor.Enabled();
		SmartDashboard::init();
	}

al::logger UltimateAscent::CreateLogger(){
	al::std_creator factory;
	factory.add_handler(smart_ptr<handler>(new al::cerr_handler()));
	smart_ptr<handler> h = smart_ptr<handler>(new al::file_handler("log"));
	factory.add_handler(h);
	return factory.spawn("UltimateAscent");
}

void UltimateAscent::Autonomous(void)
	{
//		GetWatchdog().SetEnabled(false);
		log << "Begining Autonomous" << al::endl;
		
		if (IsAutonomous ()) {
			while (IsAutonomous() && ShooterAngle(potentiometer.GetAverageVoltage()) > 11.7){
				shooterAngleMotor.Set(Relay::kForward);
				SmartDashboard::PutNumber("Potentiometer",ShooterAngle(potentiometer.GetAverageVoltage()));
			}
			while (IsAutonomous() && ShooterAngle(potentiometer.GetAverageVoltage()) < 19){
				shooterAngleMotor.Set(Relay::kReverse);
				flywheelMotor.Set(1);
				SmartDashboard::PutNumber("Potentiometer",ShooterAngle(potentiometer.GetAverageVoltage()));
			}
			shooterAngleMotor.Set(Relay::kOff);
			// Waits in order to let the flywheel to get up to speed
			// Wait (2);
			// Shoots first ball
			while (IsAutonomous()){
				flywheelMotor.Set(1);
				log << "Set Launchers Out";
				SetLauncherOut();
				// Leaves piston out for .25 seconds
				Wait(0.25);
				log << "Set Launchers In";
				SetLauncherIn();
				// Waits for the flywheel to get up to speed between shots
				Wait(2.5);
				
				// Shoots second ball
				log << "Set Launchers Out";
				SetLauncherOut();
				// Leaves piston out for .25 seconds
				Wait(0.25);
				log << "Set Launchers In";
				SetLauncherIn();
				
				// Waits for the flywheel to get up to speed between shots
				Wait(2.5);
				
				// Shoots third ball
				log << "Set Launchers Out";
				SetLauncherOut();
				// Leaves piston out for .25 seconds
				Wait(0.25);
				log << "Set Launchers In";
				SetLauncherIn();
				// Sets the flywheel speed to zero before teleop
				flywheelMotor.Set(0);
			}
		}
		
		log << "Ending Autonomous\n";
	}


void UltimateAscent::OperatorControl(void)
	{
		log << "Begining Operator Control\n";
//		myRobot.SetSafetyEnabled(false);
		while (IsOperatorControl())
		{
			Drive();
			Scoop();
			Shoot();
			// Run the compressor until it reaches a certain pressure
			if ( !compressor.GetPressureSwitchValue()){ 
				compressor.Start();
			}
			else{
				compressor.Stop();
			}
			// Keep track of frisbees in robot
			bool frisbee1 = false;
			bool frisbee2 = false;
			bool frisbee3 = false;
			bool frisbee4 = false;
			
			if (frisbeeCount >= 1) {
				frisbee1 = true;
			}
			if (frisbeeCount >= 2) {
				frisbee2 = true;
			}
			if (frisbeeCount >= 3) {
				frisbee3 = true;
			}
			if (frisbeeCount == 4) {
				frisbee4 = true;
			}
			SmartDashboard::PutBoolean("Frisbee1", frisbee1);
			SmartDashboard::PutBoolean("Frisbee2", frisbee2);
			SmartDashboard::PutBoolean("Frisbee3", frisbee3);
			SmartDashboard::PutBoolean("Frisbee4", frisbee4);
			SmartDashboard::PutNumber("Potentiometer",ShooterAngle(potentiometer.GetAverageVoltage()));
			SmartDashboard::PutNumber("Fly Wheel Motor PID", flywheelMotor.Get());
			SmartDashboard::PutNumber("Fly Wheel RPM", flywheelEncoder.GetRate());
		}
	}

void UltimateAscent::Test() {
}


// Desensitize Joystick
float UltimateAscent::ConvertAxis(float input){
	if (input >= 0.05) {
		return pow((input), 3);
	}
	else if (input <= -0.05) {
		return (pow((input), 3));
	}
	else {
		return 0;
	}
}


void UltimateAscent::Drive(){
	
	log << "Begining Drive\n";
	// Joystick Axis Inputs
	float xOutput = ConvertAxis(stick1.GetX());
	float yOutput = ConvertAxis(stick1.GetY());
	float twistOutput = ConvertAxis(stick1.GetTwist()) / 2;
	
	
	//Grippy Deployment
	if (stick1.GetRawButton(GRIPPIES_DOWN_BUTTON)){
		log << "Grippies down button pressed\n";
		xOutput = 0;
		solenoid1.Set(false);
		solenoid2.Set(true);
		solenoid3.Set(false);
		solenoid4.Set(true);
	}
	else{
		solenoid1.Set(true);
		solenoid2.Set(false);
		solenoid3.Set(true);
		solenoid4.Set(false);
	}
	//Drive with Mecanum Style
	myRobot.MecanumDrive_Cartesian(xOutput, yOutput, twistOutput);
}

void UltimateAscent::Scoop(){
	log << "Begining Scoop\n";
	static bool scoopState = false;
	//Rising edge detector variables for Toggle Button and Frisbee Light Sensor
	static bool previousFrisbeeLightValue = false;
	bool currentFrisbeeLightValue = frisbeeLightSensor.Get();
	
	//Count Frisbees if Light Sensor is tripped on Rising Edge
//	if (currentFrisbeeLightValue == true && previousFrisbeeLightValue == false){
//		frisbeeCount ++;
//	}
	
	//Toggle the state of the Scoop. Deployed or Undeployed
	if (stick1.GetRawButton(6)){
		scoopState = false;
	}
	else if(stick1.GetRawButton(4)){
		scoopState = true;
	}
	if(stick1.GetRawButton(SCOOP_UP_BUTTON)){
		scoopState = false;
		scoopSolenoid1.Set(true);
		scoopSolenoid2.Set(false);
	}
	else if(stick1.GetRawButton(SCOOP_DOWN_BUTTON)){
		scoopState = true;
		scoopSolenoid1.Set(false);
		scoopSolenoid2.Set(true);		
	}
	
	if (stick1.GetRawButton(SCOOP_BUTTON)){
		brushMotor.Set(1);
//		rearRightMotor.Set(1);
	}
	else if (stick1.GetRawButton(SCOOP_REVERSE_BUTTON)){
		brushMotor.Set(-1);
//		rearRightMotor.Set(-1);
	}
	else {
		brushMotor.Set(0);
//		rearRightMotor.Set(0);
	}
	
	//Deployment angle at 11.7
	//Runs the elevator if Scoop is deployed.
	if (scoopState){
		elevatorMotor.Set(-0.8);
	}
	else{
		elevatorMotor.Set(0);
	}

	//Sets the scoop solenoids to the current state
//	scoopSolenoid1.Set(scoopState);
//	scoopSolenoid2.Set(scoopState);
	//Rising Edge detector. Sets previous values to thier current values.
	previousFrisbeeLightValue = currentFrisbeeLightValue;
}

const double startSpeed = 200;

void UltimateAscent::Shoot() {
	static bool flywheelState = false;
	log << "Begining Shoot\n";
	if (stick2.GetRawButton(ANGLE_UP_BUTTON)){
		shooterAngleMotor.Set(Relay::kForward);
	}
	else if (stick2.GetRawButton(ANGLE_DOWN_BUTTON)){
		shooterAngleMotor.Set(Relay::kReverse);
	}
	else{
		shooterAngleMotor.Set(Relay::kOff);
	}
	// waitForLeaving is used as a buffer between shots
	static bool waitForLeaving = true;
	// Both are used for rising edge detector
	bool triggerButton = false;
	static bool priorTriggerButton = false;
	
	//Actuators
	if (stick2.GetRawButton(FIRE_BUTTON) && !priorTriggerButton) {
		// Fire frisbee if the button is pressed
		stopwatch.Reset();
		stopwatch.Start();
		waitForLeaving = false;
		triggerButton = true;
		frisbeeCount --;
		
		log << "Frisbees - 1\n";
	}
	
	// Leaves shooter out for 0.25 seconds
	if (stopwatch.Get() >= 0.25 || waitForLeaving == true) {
		if (stopwatch.Get() >= 0.5 || waitForLeaving == true) {
			log << "set launchers to false\n";
			SetLauncherFalse();
		}
		else {
			log << "launcherOut set to true\n";
			SetLauncherIn();
		}
	}
	else {
		log << "launcherIn set to true\n";
		SetLauncherOut();
	}
	
	if (stopwatch.Get() >= 5) {
		stopwatch.Stop();
	}
	//Makes 1 time button work
	if (triggerButton) {
		priorTriggerButton = true;
		triggerButton = false;
	}
	else {
		priorTriggerButton = false;
	}
	
	
	// double currentSpeed = 0;
	static double desiredSpeed = 0;
	
	// Turn flywheels on and off
	if (stick2.GetRawButton(FLYWHEEL_ON_BUTTON)) {
		log << "flyWheel desiredSpeed set to startSpeed\n";
		flywheelState = true;
		flywheelSpeed.Enable();
		desiredSpeed = startSpeed;
	}
	else if (stick2.GetRawButton(FLYWHEEL_OFF_BUTTON)) {
		log << "flyWheel desiredSpeed set to 0\n";
		flywheelState = false;
		flywheelSpeed.Disable();
		desiredSpeed = 0;
	}
	if (flywheelState){
		desiredSpeed = 40;
		flywheelSpeed.SetSetpoint(desiredSpeed);
//		flywheelMotor.Set(1);
	}
//	else {
//		flywheelMotor.Set(0);
//	}
}

void UltimateAscent::SetLauncherOut()
{
	launcherOut.Set(true);
	launcherIn.Set(false);
}

void UltimateAscent::SetLauncherIn()
{
	launcherIn.Set(true);
	launcherOut.Set(false);
}

void UltimateAscent::SetLauncherFalse()
{
	launcherIn.Set(false);
	launcherOut.Set(false);
}
float UltimateAscent::ShooterAngle(float pot)
{
	// Change the potentiometer reading to an angle measurement
	return (pot * 7.2156) - 5.525;
}

START_ROBOT_CLASS(UltimateAscent);

