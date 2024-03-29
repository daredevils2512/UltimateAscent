#include <memory>
#include <cmath>
#include "UltimateAscent.h"

const double UltimateAscent::PYRAMID_ANGLE = 20.4;
const double UltimateAscent::FEEDER_ANGLE = 15.1;
const float UltimateAscent::PYRAMID_SPEED = 75;
const float UltimateAscent::FEEDER_SPEED = 85;
const double UltimateAscent::LAUNCHER_WAIT_TIME = 0.25;

UltimateAscent::UltimateAscent(void):
		// these must be initialized in the same order as they are declared in the header file.
		gameTimer(),
		timer(),
		flywheelTimer(),
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
		counter(FLWYHEEL_LIGHT_SENSOR_SIDECAR, FLYWHEEL_LIGHT_SENSOR_PWM),
		flywheelLightSensor(FLWYHEEL_LIGHT_SENSOR_SIDECAR, FLYWHEEL_LIGHT_SENSOR_PWM),
		frisbeeLightSensor(FRISBEE_LIGHT_SENSOR_SIDECAR, FRISBEE_LIGHT_SENSOR_PWM),
		flywheelEncoder(flywheelLightSensor),
		leftMotorEncoder(LEFT_MOTOR_ENCODER_PWM_A, LEFT_MOTOR_ENCODER_PWM_B),
		rightMotorEncoder(RIGHT_MOTOR_ENCODER_PWM_A, RIGHT_MOTOR_ENCODER_PWM_B),
		stopwatch(),
		pidOutput(flywheelMotor),
		flywheelSpeed(0.750625, 0.03852941, 0, &flywheelEncoder, &pidOutput, 0.125),
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
		flywheelSpeed.Enable();
		flywheelSpeed.SetInputRange(0, 144);
		flywheelSpeed.SetOutputRange(0, 64);
		compressor.Enabled();
		leftMotorEncoder.SetDistancePerPulse(1);
		rightMotorEncoder.SetDistancePerPulse(1);
		SmartDashboard::init();
	}


void UltimateAscent::Autonomous(void)
	{
	flywheelTimer.Start();
	counter.Start();
	flywheelSpeed.Enable();
		if (IsAutonomous ()) {
			// autonAngle is the angle from which we shoot
			// angleStop is for when we shoot the first frisbee
//			bool angleStop = false;
			double autonAngle;
			double currentCounter;
			double previousCounter;
			double previousCounter2;
			leftMotorEncoder.Start();
			leftMotorEncoder.Reset();
			rightMotorEncoder.Start();
			rightMotorEncoder.Reset();
			// Determines autonAngle based on joystick 2's throttle
			autonAngle = 20.4;
			// Raises shooter to allow upper to deploy
			while (IsAutonomous() && ShooterAngle(potentiometer.GetAverageVoltage()) > 12.4){
				shooterAngleMotor.Set(Relay::kReverse);
				SmartDashboard::PutNumber("Potentiometer",ShooterAngle(potentiometer.GetAverageVoltage()));
			}
			// Lowers shooter to the Autonomous shooting angle
			while (IsAutonomous() && ShooterAngle(potentiometer.GetAverageVoltage()) < autonAngle){
//				if(angleStop == false && ShooterAngle(potentiometer.GetAverageVoltage()) > autonAngle - 1) {
//					shooterAngleMotor.Set(Relay::kOff);
//					AutonomousShoot();
//					angleStop = true;
//					Wait(1.5);
//				}
				if(flywheelTimer.Get() >= 0.125){
					currentCounter = counter.Get() * 8;
					flywheelEncoder.SetRotations(static_cast<int>((previousCounter2 * 0.1) + (previousCounter * 0.3) + (currentCounter * 0.6)));
					previousCounter2 = previousCounter;
					previousCounter = currentCounter;
					flywheelTimer.Reset();
					counter.Reset();
				}
				shooterAngleMotor.Set(Relay::kForward);
				flywheelSpeed.SetSetpoint(PYRAMID_SPEED);
				SmartDashboard::PutNumber("Potentiometer",ShooterAngle(potentiometer.GetAverageVoltage()));
			}
			shooterAngleMotor.Set(Relay::kOff);
			flywheelEncoder.SetRotations(40);
			// Shoots 3 frisbees, actuates the solenoids 4 times in case of a jam
			for (int i = 0; i < 3; i++) {
				if(i == 0) {
					Wait(1);
				}
//				if(flywheelTimer.Get() >= 0.125){
//					currentCounter = counter.Get() * 8;
//					flywheelEncoder.SetRotations(static_cast<int>((previousCounter2 * 0.1) + (previousCounter * 0.3) + (currentCounter * 0.6)));
//					previousCounter2 = previousCounter;
//					previousCounter = currentCounter;
//					flywheelTimer.Reset();
//					counter.Reset();
//				}
				AutonomousShoot();
//				if (i != 3) {
					Wait(1.5);
//				}
			}
			// Sets the flywheel speed to zero before teleop
			flywheelSpeed.SetSetpoint(0);
			flywheelSpeed.Disable();
			// Reset drive encoders
			rightMotorEncoder.Reset();
			leftMotorEncoder.Reset();
			// Turn away from pyramid
			while(rightMotorEncoder.GetRaw() <= 530 && IsAutonomous()) {
				myRobot.TankDrive(.5, .5, false);
				SmartDashboard::PutNumber("Drive Encoder", rightMotorEncoder.GetRaw());
			}
			rightMotorEncoder.Reset();
			leftMotorEncoder.Reset();
			//Back up to the middle line
			while(rightMotorEncoder.GetRaw() <= 1750 && IsAutonomous()) {
				myRobot.TankDrive(-.5,.5,false);
				SmartDashboard::PutNumber("Drive Encoder", rightMotorEncoder.GetRaw());
			}
			rightMotorEncoder.Reset();
			leftMotorEncoder.Reset();
			// turn to face frisbees
			while(rightMotorEncoder.GetRaw() >= -255 && IsAutonomous()) {
				myRobot.TankDrive(-.5, -.5, false);
				SmartDashboard::PutNumber("Drive Encoder", rightMotorEncoder.GetRaw());
			}
			rightMotorEncoder.Reset();
			leftMotorEncoder.Reset();
//			scoopSolenoid1.Set(false);
//			scoopSolenoid2.Set(true);
//			elevatorMotor.Set(-0.8);
//			brushMotor.Set(1);
//			while (IsAutonomous() && ShooterAngle(potentiometer.GetAverageVoltage()) > 14){
//				shooterAngleMotor.Set(Relay::kReverse);
//				SmartDashboard::PutNumber("Potentiometer",ShooterAngle(potentiometer.GetAverageVoltage()));
//			}
			// Drive straight away
//			while(leftMotorEncoder.GetRaw() >= -960 && IsAutonomous()) {
//				myRobot.TankDrive(.5, .5, false);
//			}
		}
	}


void UltimateAscent::OperatorControl(void)
	{
		static int previousCounter2 = 0;
		static int previousCounter = 0;
		static int currentCounter = 0;
		static int cycles = 0;
		leftMotorEncoder.Start();
		leftMotorEncoder.Reset();
		rightMotorEncoder.Start();
		rightMotorEncoder.Reset();
		gameTimer.Start();
		timer.Start();
		flywheelTimer.Reset();
		counter.Reset();
		flywheelTimer.Start();
		counter.Start();
		flywheelSpeed.Enable();
		flywheelSpeed.SetSetpoint(FEEDER_SPEED);
		while (IsOperatorControl())
		{
			cycles++;
			Drive();
			Scoop();
			if(flywheelTimer.Get() >= 0.125){
				currentCounter = counter.Get() * 8;
				if (abs(currentCounter - previousCounter2) >= 50) {
					currentCounter = static_cast<int>(flywheelSpeed.GetSetpoint());
				}
				flywheelEncoder.SetRotations(static_cast<int>((previousCounter2 * 0.1) + (previousCounter * 0.3) + (currentCounter * 0.6)));
				previousCounter2 = previousCounter;
				previousCounter = currentCounter;
				flywheelTimer.Reset();
				counter.Reset();
			}
			Shoot();
			// Run the compressor until it reaches a certain pressure
			if ( !compressor.GetPressureSwitchValue()){ 
				compressor.Start();
			}
			else{
				compressor.Stop();
			}
			if(flywheelTimer.Get() >= 0.125){
				currentCounter = counter.Get() * 8;
				if (abs(currentCounter - previousCounter2) >= 50) {
					currentCounter = static_cast<int>(flywheelSpeed.GetSetpoint());
				}
				flywheelEncoder.SetRotations(static_cast<int>((previousCounter2 * 0.1) + (previousCounter * 0.3) + (currentCounter * 0.6)));
				previousCounter2 = previousCounter;
				previousCounter = currentCounter;
				flywheelTimer.Reset();
				counter.Reset();
			}
			if(gameTimer.Get() >= 119.9){
				solenoid1.Set(true);
				solenoid2.Set(false);
				solenoid3.Set(true);
				solenoid4.Set(false);
			}
			if(timer.Get() >= .125){
				SmartDashboard::PutNumber("Potentiometer",ShooterAngle(potentiometer.GetAverageVoltage()));
				SmartDashboard::PutNumber("FlyWheel RPS", flywheelEncoder.GetRate());
				SmartDashboard::PutNumber("Right Drive Encoder", rightMotorEncoder.GetRaw());
				SmartDashboard::PutNumber("PID SetPoint", (((stick2.GetThrottle() - 1) / -2) * 144));
//				SmartDashboard::PutNumber("Cycles", cycles);
//				SmartDashboard::PutNumber("Drive Encoder", leftMotorEncoder.GetRaw());
				timer.Reset();
			}
		}
	}

void UltimateAscent::Test() {
}


// Desensitize Joystick
float UltimateAscent::ConvertAxis(float input){
	if (input >= 0.05) {
//		return input * 1.5;
		return pow((input), 3);
	}
	else if (input <= -0.05) {
//		return input * 1.5;
		return (pow((input), 3));
	}
	else {
		return 0;
	}
}


void UltimateAscent::Drive(){
	
	// Joystick Axis Inputs
	float xOutput;
	float yOutput = ConvertAxis(stick1.GetY());
	float twistOutput = ConvertAxis(stick1.GetTwist()) / 1.5;
	if (stick1.GetRawButton(12)) {
		yOutput = -1;
	}
	if(stick1.GetRawAxis(4) < 0) {
		xOutput = ConvertAxis(stick1.GetX()) / 2;
		twistOutput = ConvertAxis(stick1.GetTwist()) / 2;
	}
	else {
		xOutput = ConvertAxis(stick1.GetX());
	}
	
	//Grippy Deployment
	if (stick1.GetRawButton(GRIPPIES_DOWN_BUTTON)){
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
	static bool scoopState = false;
	//Rising edge detector variables for Toggle Button and Frisbee Light Sensor
	static bool previousScoopButton = false;
	static bool previousFrisbeeLightValue = false;
	bool currentFrisbeeLightValue = frisbeeLightSensor.Get();
	bool currentScoopButton = stick1.GetRawButton(SCOOP_TOGGLE_BUTTON);
	
	
	//Toggle the state of the Scoop. Deployed or Undeployed
	if (stick1.GetRawButton(HOTWHEELS_OFF_BUTTON)){
		scoopState = false;
	}
	else if(stick1.GetRawButton(HOTWHEELS_ON_BUTTON)){
		scoopState = true;
	}
	// Toggle for scoop deploy
	if(currentScoopButton == true && previousScoopButton == false){
		if(scoopSolenoid1.Get() == false){
			scoopState = false;
			scoopSolenoid1.Set(true);
			scoopSolenoid2.Set(false);			
		}
		else{
			scoopState = true;
			scoopSolenoid1.Set(false);
			scoopSolenoid2.Set(true);
		}

	}
	if (stick1.GetRawButton(BRUSH_BUTTON)){
		brushMotor.Set(1);
	}
	else if (stick1.GetRawButton(BRUSH_REVERSE_BUTTON)){
		brushMotor.Set(-1);
	}
	else {
		brushMotor.Set(0);
	}
	
	//Deployment angle at 11.7
	//Runs the elevator if Scoop is deployed.
	if (scoopState){
		elevatorMotor.Set(-0.8);
	}
	else{
		elevatorMotor.Set(0);
	}

	//Rising Edge detector. Sets previous values to thier current values.
	previousScoopButton = currentScoopButton;
	previousFrisbeeLightValue = currentFrisbeeLightValue;
}

void UltimateAscent::Shoot() {
//	static bool flywheelState = true;
	static bool stowOn = false;
	static bool goToAngleReached = true;
	static float desiredAngle = 0;
	// changes shooting angle
	if (stick2.GetRawButton(ANGLE_UP_BUTTON) && ShooterAngle(potentiometer.GetAverageVoltage()) >= 10){
		shooterAngleMotor.Set(Relay::kReverse);
		goToAngleReached = true;
	}
	else if (stick2.GetRawButton(ANGLE_DOWN_BUTTON)  && ShooterAngle(potentiometer.GetAverageVoltage()) <= 22.47){
		shooterAngleMotor.Set(Relay::kForward);
		goToAngleReached = true;
	}
//	else if ((stick1.GetRawButton(STOW_BUTTON_1) || stick2.GetRawButton(STOW_BUTTON_2)) && ShooterAngle(potentiometer.GetAverageVoltage()) <= 22.47) {
//		shooterAngleMotor.Set(Relay::kReverse);
//	}
	else{
		shooterAngleMotor.Set(Relay::kOff);
	}
	if(stick1.GetRawButton(STOW_BUTTON_1) || stick2.GetRawButton(STOW_BUTTON_2)){
		stowOn = true;
	}
	if(stick2.GetRawButton(PYRAMID_ANGLE_BUTTON)){
		goToAngleReached = false;
		desiredAngle = PYRAMID_ANGLE;
		flywheelSpeed.Enable();
		flywheelSpeed.SetSetpoint(PYRAMID_SPEED);
	}
	if(stick2.GetRawButton(FEEDER_ANGLE_BUTTON)){
		goToAngleReached = false;
		desiredAngle = FEEDER_ANGLE;
		flywheelSpeed.Enable();
		flywheelSpeed.SetSetpoint(FEEDER_SPEED);
	}
	if (goToAngleReached == false){
		goToAngleReached = GoToAngle(desiredAngle);
	}
	if (stick2.GetRawButton(ADJ_BUTTON)){
		flywheelSpeed.Enable();
		flywheelSpeed.SetSetpoint(((stick2.GetThrottle() - 1) / -2) * 144);
//		flywheelMotor.Set(1);
	}
	if(stowOn){
		goToAngleReached = true;
		if(ShooterAngle(potentiometer.GetAverageVoltage()) <= 22.47){
			shooterAngleMotor.Set(Relay::kForward);
		}
		else{
			stowOn = false;
			shooterAngleMotor.Set(Relay::kOff);
		}
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
	}
	
	// Leaves shooter out for a short time
	if (stopwatch.Get() >= LAUNCHER_WAIT_TIME || waitForLeaving == true) {
		// TODO: Check whether 0.5 should be replaced by LAUNCHER_WAIT_TIME * 2
		if (stopwatch.Get() >= 0.5 || waitForLeaving == true) {
			SetLauncherFalse();
		}
		else {
			SetLauncherIn();
		}
	}
	else {
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
	
	// Turn flywheels on and off
	if (stick2.GetRawButton(FLYWHEEL_ON_BUTTON)) {
//		flywheelState = true;
		flywheelSpeed.Enable();
		flywheelSpeed.SetSetpoint(FEEDER_SPEED);
	}
	else if (stick2.GetRawButton(FLYWHEEL_OFF_BUTTON)) {
//		flywheelState = false;
		flywheelSpeed.SetSetpoint(0);
		flywheelSpeed.Disable();
	}
	// Set to speed of throttle on Joystick 2
//	if (!flywheelState){
//		if (stick2.GetRawButton(ADJ_BUTTON)) {
//			flywheelMotor.Set((stick2.GetRawAxis(3) - 1) / -2);
//		}
//		else {
//			flywheelMotor.Set(1);
//		}
//	
//		flywheelSpeed.SetSetpoint(0);
//	}
//	else{
//		flywheelSpeed.SetSetpoint(FEEDER_SPEED);
//	}
//	else {
//		flywheelMotor.Set(0);
//	}
}

// Functions for the shooter solenoids
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

// Shoots a frisbee leaving the solenoids out for a short period
void UltimateAscent::AutonomousShoot ()
{
	SetLauncherOut();
	// Leaves piston out for .25 seconds
	Wait(LAUNCHER_WAIT_TIME);
	SetLauncherIn();
}

float UltimateAscent::ShooterAngle(float pot)
{
	// Change the potentiometer reading to an angle measurement
	return (pot * 7.2156) - 5.525;
}

bool UltimateAscent::GoToAngle(double desiredAngle) {
	static bool aboveDesiredAngle = false;
	static bool reachedDesiredAngle = false;
	if (ShooterAngle(potentiometer.GetAverageVoltage()) > desiredAngle + 0.5){
		aboveDesiredAngle = false;
	}
	else if (ShooterAngle(potentiometer.GetAverageVoltage()) < desiredAngle - 0.5) {
		aboveDesiredAngle = true;
	}
	if(aboveDesiredAngle){
		shooterAngleMotor.Set(Relay::kForward);
	}
	else if(aboveDesiredAngle == false){
		shooterAngleMotor.Set(Relay::kReverse);
	}
	if (ShooterAngle(potentiometer.GetAverageVoltage()) < desiredAngle + 0.5 
			&& ShooterAngle(potentiometer.GetAverageVoltage()) > desiredAngle - 0.5) {
		shooterAngleMotor.Set(Relay::kOff);
		reachedDesiredAngle = true;
	}
	else {
		reachedDesiredAngle = false;
	}
	return reachedDesiredAngle;
}

START_ROBOT_CLASS(UltimateAscent);

