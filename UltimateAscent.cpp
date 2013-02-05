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
		scoopSolenoid(SCOOP_SOLENOID_SIDECAR, SCOOP_SOLENOID_PWM),
		launcherIn(LAUNCHER_IN_SIDECAR, LAUNCHER_IN_PWM),
		launcherOut(LAUNCHER_OUT_SIDECAR, LAUNCHER_OUT_PWM),
		flywheelLightSensor(FLWYHEEL_LIGHT_SENSOR_SIDECAR, FLYWHEEL_LIGHT_SENSOR_PWM),
		frisbeeLightSensor(FRISBEE_LIGHT_SENSOR_SIDECAR, FRISBEE_LIGHT_SENSOR_PWM),
		flywheelEncoder(flywheelLightSensor),
		leftMotorEncoder(LEFT_MOTOR_ENCODER_PWM_A, LEFT_MOTOR_ENCODER_PWM_B),
		rightMotorEncoder(RIGHT_MOTOR_ENCODER_PWM_A, RIGHT_MOTOR_ENCODER_PWM_B),
		stopwatch(),
		pidOutput(flywheelMotor),
		flywheelSpeed(0, 0, 0, &flywheelEncoder, &pidOutput),//TODO:Unfinished
		myRobot(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor),
		stick1(1),
		stick2(2),
		potentiometer(POTENTIOMETER_SIDECAR, POTENTIOMETER_PWM)
	{
		myRobot.SetExpiration(0.1);
		stick1.SetAxisChannel(Joystick::kTwistAxis, 3);
		stick1.SetAxisChannel(Joystick::kThrottleAxis, 4);
		flywheelSpeed.Enable();
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

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
void UltimateAscent::Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		log << "Begining Autonomous" << al::endl;
		
		if (IsAutonomous ()) {
			flywheelMotor.Set(1);
			Wait (2);
			// Shoots first ball
			SetLauncherOut();
			// Leaves piston out for .25 seconds
			Wait(0.25);
			SetLauncherIn();
			
			// Waits 1.5 seconds until shooting again
			Wait(1.5);
			
			// Shoots second ball
			SetLauncherOut();
			// Leaves piston out for .25 seconds
			Wait(0.25);
			SetLauncherIn();
			
			// Waits 1.5 seconds until shooting again
			Wait(1.5);
			
			// Shoots third ball
			SetLauncherOut();
			// Leaves piston out for .25 seconds
			Wait(0.25);
			SetLauncherIn();
			
			flywheelMotor.Set(0);
		}
		
		log << "Ending Autonomous\n";
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
void UltimateAscent::OperatorControl(void)
	{
		log << "Begining Operator Control" << endl;
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			Drive();
			Scoop();
			Shoot();
			if ( !compressor.GetPressureSwitchValue()){
				compressor.Start();
			}
			else{
				compressor.Stop();
			}
			SmartDashboard::PutNumber("Count" ,frisbeeCount);
			SmartDashboard::PutNumber("Potentiometer",potentiometer.GetVoltage());
		}
	}

/**
 * Runs during test mode
 */
void UltimateAscent::Test() {
}

float UltimateAscent::ConvertAxis(float input){
	if (input >= 0.05) {
		return pow((input*0.75f), 2);
	}
	else if (input <= -0.05) {
		return pow((input*0.75f), 2)*-1;
	}
	else {
		return 0;
	}
}

void UltimateAscent::Drive(){
	
	log << "Begining Drive\n";
	
	float xOutput = ConvertAxis(stick1.GetX());
	float yOutput = ConvertAxis(stick1.GetY());
	float twistOutput = ConvertAxis(stick1.GetTwist());
	
	if (stick1.GetRawButton(GRIPPIES_DOWN_BUTTON)){
		xOutput = 0;
		solenoid1.Set(true);
		solenoid2.Set(true);
	}
	else{
		solenoid1.Set(false);
		solenoid2.Set(false);				
	}
	myRobot.MecanumDrive_Cartesian(xOutput, yOutput, twistOutput); // drive with arcade style (use right stick)
}

void UltimateAscent::Scoop(){
	log << "Begining Scoop\n";
	static bool scoopState = false;
	static bool previousScoopButton = false;
	static bool previousFrisbeeLightValue = false;
	bool currentScoopButton = stick1.GetRawButton(SCOOP_BUTTON);
	bool currentFrisbeeLightValue = frisbeeLightSensor.Get();
	
	if (currentFrisbeeLightValue == true && previousFrisbeeLightValue == false){
		frisbeeCount ++;
	}
	
	if (currentScoopButton == true && previousScoopButton == false){
		if (scoopState == true){
			scoopState = false;
		}
		else{
			scoopState = true;
		}
	}
	if (scoopState){
		elevatorMotor.Set(1);
		if (frisbeeCount < 4){
			brushMotor.Set(1);
		}
		else{
			brushMotor.Set(0);
		}
	}
	else{
		elevatorMotor.Set(0);
	}
	scoopSolenoid.Set(scoopState);
	previousFrisbeeLightValue = currentFrisbeeLightValue;
	previousScoopButton = currentScoopButton;
}

const double startSpeed = 200;

void UltimateAscent::Shoot() {
	log << "Begining Shoot\n";
	// waitForLeaving is used as a buffer between shots
	static bool waitForLeaving = true;
	bool triggerButton = false;
	static bool priorTriggerButton = false;
	
	//Actuators
	if (stick2.GetRawButton(FIRE_BUTTON) && !priorTriggerButton) {
		stopwatch.Reset();
		stopwatch.Start();
		waitForLeaving = false;
		triggerButton = true;
		frisbeeCount --;
		
		log << "Frisbees - 1\n";
	}
	if (stopwatch.Get() >= 0.25 || waitForLeaving == true) {
		if (stopwatch.Get() >= 0.5 || waitForLeaving == true) {
			log << "set launchers to false\n";
			SetLauncherFalse();
		}
		else {
			log << "launcherOut set to true\n";
			SetLauncherOut();
		}
	}
	else {
		log << "launcherIn set to true\n";
		SetLauncherIn();
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
	
	
//	double currentSpeed = 0;
	static double desiredSpeed = 0;
	
	if (stick2.GetRawButton(FLYWHEEL_ON_BUTTON)) { // Buttons have been initialized.
		log << "flyWheel desiredSpeed set to startSpeed\n";
		desiredSpeed = startSpeed;
	}
	else if (stick2.GetRawButton(FLYWHEEL_OFF_BUTTON)) { // Buttons have been initialized.
		log << "flyWheel desiredSpeed set to 0\n";
		desiredSpeed = 0;
	}
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

START_ROBOT_CLASS(UltimateAscent);

