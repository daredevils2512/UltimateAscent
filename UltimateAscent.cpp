#include <cmath>
#include "UltimateAscent.h"

UltimateAscent::UltimateAscent(void):
		// these must be initialized in the same order as they are declared in the header file.
		timer(),
		frontLeftMotor(FRONT_LEFT_MOTOR_SIDECAR, FRONT_LEFT_MOTOR_PWM),
		frontRightMotor(FRONT_RIGHT_MOTOR_SIDECAR, FRONT_RIGHT_MOTOR_PWM),
		rearLeftMotor(REAR_LEFT_MOTOR_SIDECAR, REAR_LEFT_MOTOR_PWM),
		rearRightMotor(REAR_RIGHT_MOTOR_SIDECAR, REAR_RIGHT_MOTOR_PWM),
		flywheelMotor(FLYWHEEL_MOTOR_SIDECAR, FLYWHEEL_MOTOR_PWM),
		solenoid1(SOLENOID1_SIDECAR, SOLENOID1_PWM),
		solenoid2(SOLENOID2_SIDECAR, SOLENOID2_PWM),
		scoopSolenoid(SCOOP_SOLENOID_SIDECAR, SCOOP_SOLENOID_PWM),
		flywheelLightSensor(FLWYHEEL_LIGHT_SENSOR_SIDECAR, FLYWHEEL_LIGHT_SENSOR_PWM),
		flywheelEncoder(flywheelLightSensor),
		pidOutput(flywheelMotor),
		flywheelSpeed(0, 0, 0, &flywheelEncoder, &pidOutput),//TODO:Unfinished
		myRobot(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor),
		stick1(1),
		stick2(2)
	{
		myRobot.SetExpiration(0.1);
		stick1.SetAxisChannel(Joystick::kTwistAxis, 3);
		stick1.SetAxisChannel(Joystick::kThrottleAxis, 4);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
void UltimateAscent::Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
void	UltimateAscent::OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			Drive();
			Scoop();
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
	static bool scoopState = false;
	static bool previousScoopButton = false;
	
	bool currentScoopButton = stick1.GetRawButton(SCOOP_BUTTON);
	if (currentScoopButton == true && previousScoopButton == false){
		if (scoopState == true){
			scoopState = false;
		}
		else{
			scoopState = true;
		}
	}
	scoopSolenoid.Set(scoopState);
	previousScoopButton = currentScoopButton;
}

const double startSpeed = 200;

void UltimateAscent::Shoot() {
//	double currentSpeed = 0;
	static double desiredSpeed = 0;
	
	if (stick2.GetRawButton(FLYWHEEL_ON_BUTTON)) { // Buttons have been initialized.
		desiredSpeed = startSpeed;
	}
	else if (stick2.GetRawButton(FLYWHEEL_OFF_BUTTON)) { // Buttons have been initialized.
		desiredSpeed = 0;
	}
}


START_ROBOT_CLASS(UltimateAscent);

