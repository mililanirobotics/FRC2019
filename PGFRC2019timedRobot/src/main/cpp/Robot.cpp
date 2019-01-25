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
void Robot::RobotInit()
{
  TimedRobot(); //Sets periodic methods to run every 0.02 seconds.
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}
void Robot::setFollowers()//Sets back and middle motors to follow the front motors.
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
<<<<<<< HEAD
	RFront.ConfigPeakOutputForward(1.0, 10);
=======
	//Motors
  RFront.ConfigPeakOutputForward(1.0, 10);
>>>>>>> eab6b19660ce371ac9afd40644e96fe66c0ad2ea
	RFront.ConfigPeakOutputReverse(-1.0, 10); //Sets RFront to power range -1.0 to 1.0
	RFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
	LFront.ConfigPeakOutputForward(1.0, 10);
	LFront.ConfigPeakOutputReverse(-1.0, 10); //Sets LFront to power range -1.0 to 1.0
	LFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
<<<<<<< HEAD
=======
	compressor.SetClosedLoopControl(true); //Opens compressor
	setFollowers(); //Sets all other drive motors to follow RFront and LFront


>>>>>>> eab6b19660ce371ac9afd40644e96fe66c0ad2ea
}

void Robot::rollerInit()
{
	rollerTalon.ConfigPeakOutputForward(1.0, 10);
	rollerTalon.ConfigPeakOutputReverse(-1.0, 10);
	rollerTalon.SetNeutralMode(NeutralMode::Brake);
}

void Robot::drivePeriodic()
{
<<<<<<< HEAD
	double leftJoystickPower = -joystickL.GetY(); //Gets y value of left joystick
=======
	//Camera
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("limelight");
	
	//Controller/Motor Power
  const double ROLLERPOWER = 0.3; //constant roller power to eject and intake
  double leftJoystickPower = -joystickL.GetY(); //Gets y value of left joystick
>>>>>>> eab6b19660ce371ac9afd40644e96fe66c0ad2ea
	double rightJoystickPower = joystickR.GetY(); //Gets y value of right joystick
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
		solenoidOne.Set(true); //Opens both L's
		solenoidTwo.Set(true);
		frc::Wait(0.02); //Delay needed so hatch doesn't hit L's
		solenoidThree.Set(true); //Pushes the pusher out
		solenoidFour.Set(true);
    frc::Wait(1); //Delay to pull back the pusher
    solenoidThree.Set(false);
    solenoidFour.Set(false);
	}

	if (gamePad1.GetRawButton(6))//If right bumper is pressed, close the L's
  {
		solenoidOne.Set(false);
		solenoidTwo.Set(false);
	}
}

void Robot::pivotPeriodic()
{
		if (gamePad1.GetRawButton(2) && limitSwitchOne.Get() == 0) //b
  {
		pivotTalon.Set(ControlMode::PercentOutput, 0.3); //Pivots the payload forward
	}

	else if (gamePad1.GetRawButton(3) && limitSwitchTwo.Get() == 0) //x
  {
		pivotTalon.Set(ControlMode::PercentOutput, -0.3); //Pivots the payload backwards
	}

	else 
  {
		pivotTalon.Set(ControlMode::PercentOutput, 0); //Sets power to 0 if unattended
	}
}

void Robot::rollerPeriodic()
{
	if (gamePad1.GetRawButton(1)) //a
  {
		rollerTalon.Set(ControlMode::PercentOutput, ROLLERPOWER); //Rolls forward, intakes
	}
		
	else if (gamePad1.GetRawButton(4))//y
  {
		rollerTalon.Set(ControlMode::PercentOutput, -ROLLERPOWER); //Rolls backwards, ejects
	}

	else
  {
		rollerTalon.Set(ControlMode::PercentOutput, 0); //Sets power to 0 when unattended
	}

<<<<<<< HEAD
}
void Robot::TeleopInit() 
{
	driveInit(); //Links back to drive initialize code block

	compressor.SetClosedLoopControl(true); //Opens compressor

	setFollowers(); //Sets all other drive motors to follow RFront and LFront

	rollerInit();
}

void Robot::TeleopPeriodic() //Teleop function that runs periodically.
{
  
	drivePeriodic(); //Links back to code block relating to drive

	solenoidPeriodic(); //Links back to code block relating to solenoids

	pivotPeriodic(); //Links back to code block relating to pivoting the payload

	rollerPeriodic(); //Links back to code block relating to the roller of the payload

=======
	if (joystickR.GetRawButton(2)
	{ //Alignment with camera
  	double targetOffsetAngle_Horitzontal = table->GetNumber("tx",0.0);
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
>>>>>>> eab6b19660ce371ac9afd40644e96fe66c0ad2ea
	//The motors will be updated every 5ms
	frc::Wait(0.005);
}

void Robot::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
