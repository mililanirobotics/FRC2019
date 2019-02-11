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

double EJECTROLLERPOWER = 1.0;

void Robot::RobotInit()
{
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
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
void Robot::AutonomousInit()
{
  ///@todo Lon  Remove these unsued code, chooser, autoselected, etc.

}

void Robot::AutonomousPeriodic()
{
  ///@todo Lon you need to run code during autonomous
  TeleopInit();
  TeleopPeriodic();
}
void Robot::rollerInit()
{

  rollerTalon.ConfigPeakOutputForward(1.0, 10); //Max roller power forward = 1.0
  rollerTalon.ConfigPeakOutputReverse(-1.0, 10); //Max roller power reverse = -1.0
  rollerTalon.SetNeutralMode(NeutralMode::Brake); //Sets to brake when at 0 power

}

void Robot::hatchInit()
{

  compressor.SetClosedLoopControl(true); //Opens compressor

}

void Robot::pivotInit()
{

  pivotTalon.ConfigPeakOutputForward(1.0, 10); //Max roller power forward = 1.0
  pivotTalon.ConfigPeakOutputReverse(-1.0, 10); //Max pivot power reverse = -1.0
  pivotTalon.SetNeutralMode(NeutralMode::Brake); //brakes when power = 0

}

void Robot::setFollowers()

{
  //Follows talon with canID 12, LFront
  LBack.Set(ControlMode::Follower, 12); 
  LMiddle.Set(ControlMode::Follower, 12);
  //Follows talon with canID 13, RFront
  RBack.Set(ControlMode::Follower, 13);
  RMiddle.Set(ControlMode::Follower, 13);

}

void Robot::driveInit()

{

  RFront.ConfigPeakOutputForward(1.0, 10);
	RFront.ConfigPeakOutputReverse(-1.0, 10);	//Sets RFront to power range -1.0 to 1.0
	RFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
	LFront.ConfigPeakOutputForward(1.0, 10);
	LFront.ConfigPeakOutputReverse(-1.0, 10);	//Sets LFront to power range -1.0 to 1.0
	LFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
  setFollowers(); //Sets motors to follow RFront and LFront

}

void Robot::TeleopInit()
{

  rollerInit(); //Sets max/min power and brakemode

  hatchInit(); //Links back to function that turns on compressor

  pivotInit(); //Links back to the function that sets pivot power limitations

  driveInit(); //Links back to function that sets power limitations and followers

  timer.Start(); //timer for camera
}


void Robot::rollerPeriodic()
{

  rollerEjectButton = gamePad1.GetRawAxis(2);
  rollerIntakeButton = gamePad1.GetRawAxis(3);

  if (rollerEjectButton > 0.1) //Checks to see if left trigger is pressed
  {

    rollerTalon.Set(ControlMode::PercentOutput, EJECTROLLERPOWER);//Moves the roller forward used for ejecting cargo
  
  }
  
  else if (rollerIntakeButton > 0.1) //Checks to see if right trigger is pressed
  {

    rollerTalon.Set(ControlMode::PercentOutput, -rollerIntakeButton);//Moves the roller backward used for intake cargo
  
  }
  
  else
  {

    rollerTalon.Set(ControlMode::PercentOutput, 0);//Stopping the Rollers Motors
  
  }

}

void Robot::hatchPeriodic()
{
  
  ejectButton = gamePad1.GetRawButton(5); //Left Bumper
  grabButton = gamePad1.GetRawButton(6); //Right Bumper
  openButton = gamePad1.GetRawButton(7); //Select  
  if (ejectButton) //Checks to see if Left Bumper is pressed
  {

    topFinger.Set(true); //Opens the top fingers
    bottomFinger.Set(true); //Opens the bottom fingers
    frc::Wait(0.02); //Delay to prevent hatch from hitting the fingers
    rightPusher.Set(true); //Pushes out right pusher
    leftPusher.Set(true); //Pushes out left pusher
    frc::Wait(1); //Delay before pulling back, prevents premature retraction
    rightPusher.Set(false); //Pulls in right pusher
    leftPusher.Set(false); //Pulls in left pusher
  
  } 

  else if (grabButton) //Checks to see if Right Bumper is pressed
  {

    topFinger.Set(false); //Closes top finger
    bottomFinger.Set(false); //Closes bottom finger
  
  }

  else if (openButton)
  {

    topFinger.Set(true);
    bottomFinger.Set(true);
  
  }
///@todo Lon you sure you don't need a hidden(hard to execute) way of just opening the fingers?
}
bool Robot::inRange(int targetDegrees, double currentDegrees, double errorMargin)
{
  //Checks to see if the angle is in the range
  if (targetDegrees - errorMargin < currentDegrees && targetDegrees +errorMargin > currentDegrees)
  {

    return true;
  
  }
  else 
  {

    return false;
  
  }
}
void Robot::goToRange(int targetValue, double currentValue, double errorValue)
{
      if (!inRange(targetValue, currentValue, errorValue) && currentValue < targetValue) //Checks if it's past the angle
    {

      pivotBrake.Set(frc::DoubleSolenoid::Value::kForward); 
      pivotTalon.Set(ControlMode::PercentOutput, -1.0); 

    }

    else if (!inRange(targetValue, currentValue, errorValue) && currentValue > targetValue)
    {

      pivotBrake.Set(frc::DoubleSolenoid::Value::kForward);
      pivotTalon.Set(ControlMode::PercentOutput, 1.0);

    }

    else
    {

      pivotBrake.Set(frc::DoubleSolenoid::Value::kReverse);
      pivotTalon.Set(ControlMode::PercentOutput, 0);

    }

}
void Robot::pivotPeriodic()
{

  //Bigger number position, lower actual position
  ///@todo Lon Move this to smartdashboard
  //frc::SmartDashboard::PutNumber(wpi::StringRef, pivotPosition);
  pivotDownButton = gamePad1.GetRawButtonPressed(1);
  pivotUpButton = gamePad1.GetRawButtonPressed(4);
  double angleRadians = atan2(pivotAccel.GetX(), pivotAccel.GetY()); //Grabs angle in radians
  double angleDegrees = angleRadians * (180/M_PI); //Converts angle to degrees

  if (pivotUpButton && pivotPosition != 1)
  { 
    ///@todo Lon this requires you to hold the button down for less than 20 ms
    pivotPosition--; //Moves one position up
  
  }

  else if (pivotDownButton && pivotPosition != 4)
  {

    pivotPosition++; //Moves one position down
  
  }

  if (pivotPosition == 1) //Goes to starting p
  {
    ///@todo Lon this code pattern shows up more than once, should make a function
    goToRange(0, angleDegrees, 3.5);
  }
  else if (pivotPosition == 2) //Goes to 30 degrees past starting position
  {

    goToRange(-30, angleDegrees, 3.5);

  }
  else if (pivotPosition == 3) //Goes to 54 degrees past starting position
	{

		goToRange(-54, angleDegrees, 3.5);
    ///@todo Lon missing 0 power
	
  }
	else if (pivotPosition == 4) //Goes to 137 degrees past starting position
	{

		goToRange(-137, angleDegrees, 3.5);
	
  }

}

void Robot::drivePeriodic()
{
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

void Robot::cameraAlign()
{
  auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("limelight");

	inst.GetTable("limelight")->PutNumber("camMode", 0);//changes camera mode for light sensor

	inst.GetTable("limelight")->PutNumber("ledMode", 0);//Turns camera light on

	double targetOffsetAngle_Horitzontal = table->GetNumber("tx", 0.0);
	std::cout << targetOffsetAngle_Horitzontal << std::endl;
	double offsetAdjust = fabs(targetOffsetAngle_Horitzontal * 0.03);
	if (targetOffsetAngle_Horitzontal < 0)
	{
		RFront.Set(ControlMode::PercentOutput, -0.3);
		LFront.Set(ControlMode::PercentOutput, 0.3 - offsetAdjust);
	}
	else if (targetOffsetAngle_Horitzontal > 0)
	{
		RFront.Set(ControlMode::PercentOutput, -0.3 + offsetAdjust);
		LFront.Set(ControlMode::PercentOutput, 0.3);
	}
	else
	{
		RFront.Set(ControlMode::PercentOutput, -0.3);
		LFront.Set(ControlMode::PercentOutput, 0.3);
	}

}

void Robot::cameraPeriodicHatch()
{
  cameraAlign();
  if (timer.Get() > 3 && RFront.GetSelectedSensorVelocity() == 0 && LFront.GetSelectedSensorVelocity() == 0)
	  {
      
      ///@todo Lon write a function to shoot and use it in both places
		  topFinger.Set(true); //Opens both L's
		  bottomFinger.Set(true);
		  frc::Wait(0.02);				 //Delay needed so hatch doesn't hit L's
		  rightPusher.Set(true); //Pushes the pusher out
		  leftPusher.Set(true);
		  frc::Wait(1); //Delay to pull back the pusher
		  rightPusher.Set(false);
		  leftPusher.Set(false);
      ///@todo Lon where is position 30?  you should probably zero the position first
	}
}

void Robot::cameraPeriodicCargo()
{
  cameraAlign();
  double angleRadians = atan2(pivotAccel.GetX(), pivotAccel.GetY()); //Grabs angle in radians
  double angleDegrees = angleRadians * (180/M_PI); //Converts angle to degrees
  if (timer.Get() > 3 && RFront.GetSelectedSensorVelocity() == 0 && LFront.GetSelectedSensorVelocity() == 0)
	{
    //Needs to drive backwards first, get measurements from cad
    goToRange(-30, angleDegrees, 3.5);
    rollerTalon.Set(ControlMode::PercentOutput, EJECTROLLERPOWER);//Moves the roller forward used for ejecting cargo
  
  }

}

void Robot::emergencyPeriodic()
{ //In case of packet loss
  while (emergencyStop.Get() == 1)
  {
    ///@todo Lon if you're going to print, put a delay in here
    std::cout << "Stopped" << std::endl; //Prints continuously
    RFront.Set(ControlMode::PercentOutput, 0); //Stops all right motors
    LFront.Set(ControlMode::PercentOutput, 0); //Stops all left motors
    compressor.SetClosedLoopControl(false); //Closes compressor
    rollerTalon.Set(ControlMode::PercentOutput, 0); //Stops the roller motor
    pivotTalon.Set(ControlMode::PercentOutput, 0); //Stops the pivot motor
    timer.Stop(); //Stops the timer
  }//DO NOT DISABLE WHILE EMERGENCY STOP IS TRUE
}

void Robot::TeleopPeriodic()
{
  auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("limelight");

  if (joystickR.GetRawButton(2) || joystickL.GetRawButton(2))
  {
    cameraPeriodicHatch(); //Links back to camera code
  }
  else if (joystickR.GetRawButton(3) || joystickL.GetRawButton(3))
  {
    cameraPeriodicCargo();
  }
  else if (joystickL.GetRawButton(10) || joystickR.GetRawButton(10))
  {
    timer.Reset();

    inst.GetTable("limelight")->PutNumber("camMode", 0);//changes camera mode for light sensor

		inst.GetTable("limelight")->PutNumber("ledMode", 0);//Turns camera light on
	
  }
  else
  {
    timer.Reset();

    rollerPeriodic(); //Links back to roller code

    hatchPeriodic(); //Links back to solenoid fingers and pusher code for hatch

    pivotPeriodic(); //Links back to position code

    drivePeriodic(); //Links back to drive code and slowmode

   // emergencyPeriodic(); //Links back to emergency stop code
  }

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
