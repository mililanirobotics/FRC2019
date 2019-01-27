/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTable.h"
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

const double ROLLERPOWER = 1.0; //constant roller power to eject and intake
const int ticksPerRot = 1;			//Needs to be tested and set
void Robot::RobotInit()
{
	TimedRobot(); //Sets periodic methods to run every 0.02 seconds.
	m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}
void Robot::setFollowers() //Sets back and middle motors to follow the front motors.
{
	LBack.Set(ControlMode::Follower, 12);
	LMiddle.Set(ControlMode::Follower, 12);
	RBack.Set(ControlMode::Follower, 13);
	RMiddle.Set(ControlMode::Follower, 13);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() //Robot method that runs periodically
{
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() //Initialization of autonomous
{
	m_autoSelected = m_chooser.GetSelected();
	// m_autoSelected = SmartDashboard::GetString("Auto Selector",
	//     kAutoNameDefault);
	std::cout << "Auto selected: " << m_autoSelected << std::endl;

	if (m_autoSelected == kAutoNameCustom)
	{
		// Custom Auto goes here
	}
	else
	{
		// Default Auto goes here
	}
}

void Robot::AutonomousPeriodic() //Autonomous that runs periodically
{
	if (m_autoSelected == kAutoNameCustom)
	{
		// Custom Auto goes here
	}
	else
	{
		// Default Auto goes here
	}
}
void Robot::driveInit()
{
	RFront.ConfigPeakOutputForward(1.0, 10);
	//Motors
	RFront.ConfigPeakOutputForward(1.0, 10);
	RFront.ConfigPeakOutputReverse(-1.0, 10);	//Sets RFront to power range -1.0 to 1.0
	RFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
	LFront.ConfigPeakOutputForward(1.0, 10);
	LFront.ConfigPeakOutputReverse(-1.0, 10);	//Sets LFront to power range -1.0 to 1.0
	LFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
	compressor.SetClosedLoopControl(true);		 //Opens compressor
	setFollowers();														 //Sets all other drive motors to follow RFront and LFront
}

void Robot::rollerInit()
{
	//rollerTalon.ConfigPeakOutputForward(1.0, 10);
	//rollerTalon.ConfigPeakOutputReverse(-1.0, 10);
	//rollerTalon.SetNeutralMode(NeutralMode::Brake);
}

void Robot::drivePeriodic()
{

	//Controller/Motor Power
	double leftJoystickPower = -joystickL.GetY();								//Gets y value of left joystick
	double rightJoystickPower = joystickR.GetY();								//Gets y value of right joystick
	if (joystickL.GetRawButton(1) || joystickR.GetRawButton(1)) //Slowmode if triggers on either joysticks are pressed
	{
		rightJoystickPower *= 0.7;
		leftJoystickPower *= 0.7;
	}
	RFront.Set(ControlMode::PercentOutput, rightJoystickPower); //Sets power to y axis of joysticks
	LFront.Set(ControlMode::PercentOutput, leftJoystickPower);
}

void Robot::solenoidPeriodic()
{
	if (gamePad1.GetRawButton(5)) //Left bumper is pressed.
	{
		topFinger.Set(true); //Opens both L's
		bottomFinger.Set(true);
		frc::Wait(0.02);				 //Delay needed so hatch doesn't hit L's
		rightPusher.Set(true); //Pushes the pusher out
		leftPusher.Set(true);
		frc::Wait(1); //Delay to pull back the pusher
		rightPusher.Set(false);
		leftPusher.Set(false);
	}

	if (gamePad1.GetRawButton(6)) //If right bumper is pressed, close the L's
	{
		topFinger.Set(false);
		bottomFinger.Set(false);
	}
}

void Robot::pivotPeriodic()
{
	if (gamePad1.GetRawButton(2) && limitSwitchOne.Get() == 0) //b
	{
		//pivotTalon.Set(ControlMode::PercentOutput, 0.3); //Pivots the payload forward
	}

	else if (gamePad1.GetRawButton(3) && limitSwitchTwo.Get() == 0) //x
	{
		//pivotTalon.Set(ControlMode::PercentOutput, -0.3); //Pivots the payload backwards
	}

	else
	{
		//pivotTalon.Set(ControlMode::PercentOutput, 0); //Sets power to 0 if unattended
	}
}

void Robot::rollerPeriodic()
{
	if (gamePad1.GetRawButton(1)) //a
	{
		//rollerTalon.Set(ControlMode::PercentOutput, ROLLERPOWER); //Rolls forward, intakes
	}

	else if (gamePad1.GetRawButton(4)) //y
	{
		//rollerTalon.Set(ControlMode::PercentOutput, -ROLLERPOWER); //Rolls backwards, ejects
	}

	else
	{
		//rollerTalon.Set(ControlMode::PercentOutput, 0); //Sets power to 0 when unattended
	}
}

void Robot::cameraPeriodic()
{
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("limelight");

	double targetOffsetAngle_Horitzontal = table->GetNumber("tx", 0.0);
	std::cout << targetOffsetAngle_Horitzontal << std::endl;
	if (targetOffsetAngle_Horitzontal > -1.0)
	{
		RFront.Set(ControlMode::PercentOutput, 0.2);
		LFront.Set(ControlMode::PercentOutput, -0.3);
		std::cout << "Left" << std::endl;
	}
	else if (targetOffsetAngle_Horitzontal < 1.0)
	{
		RFront.Set(ControlMode::PercentOutput, 0.3);
		LFront.Set(ControlMode::PercentOutput, -0.2);
		std::cout << "Right" << std::endl;
	}
	else
	{
		RFront.Set(ControlMode::PercentOutput, 0.3);
		LFront.Set(ControlMode::PercentOutput, -0.3);
		std::cout << "Forward" << std::endl;
	}
}
void Robot::TeleopInit()
{
	driveInit(); //Links back to drive initialize code block

	setFollowers(); //Sets all other drive motors to follow RFront and LFront

	rollerInit(); //Links back to roller config code

	compressor.SetClosedLoopControl(true);

}


void Robot::TeleopPeriodic() //Teleop function that runs periodically.
{

	//double ticksPerMsRFront = RFront.GetSelectedSensorVelocity(13); //Gets velocity in ticks / 100 ms

	//double rotPerMsRFront = ticksPerMsRFront * ticksPerRot; //Gets velocity in rotations / 100 ms

	//double rotPerMinRFront = rotPerMsRFront * 60000; //Gets velocity in rotations / minute

	//double ticks = RFront.GetSelectedSensorPosition(13);

	//std::cout << "Ticks: " << ticks << std::endl;

	auto inst = nt::NetworkTableInstance::GetDefault();
	
	auto table = inst.GetTable("limelight");

	if (joystickR.GetRawButton(2))
	{ 
		//Alignment with camera
		cameraPeriodic();
	
	}

	else
	{

		drivePeriodic(); //Links back to code block relating to drive

		solenoidPeriodic(); //Links back to code block relating to solenoids

		pivotPeriodic(); //Links back to code block relating to pivoting the payload

		rollerPeriodic(); //Links back to code block relating to the roller of the payload
	}

	//The motors will be updated every 5ms
	frc::Wait(0.005);
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
