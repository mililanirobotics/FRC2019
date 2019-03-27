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

double EJECTROLLERPOWER = 1.0; //Full power

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

void Robot::AutonomousInit()
{

  TeleopInit();

}

void Robot::AutonomousPeriodic()
{
  
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
  
  pivotPosition = 1;
  pivotTalon.ConfigPeakOutputForward(1.0, 10); //Max roller power forward = 1.0
  pivotTalon.ConfigPeakOutputReverse(-1.0, 10); //Max pivot power reverse = -1.0
  pivotTalon.SetNeutralMode(NeutralMode::Brake); //brakes when power = 0

}

void Robot::setFollowers()

{
  //Follows talon with canID 12, LFront
  LBack.Set(ControlMode::Follower, 12); 
  //Follows talon with canID 13, RFront
  RBack.Set(ControlMode::Follower, 13);
}

void Robot::driveInit()

{

  RFront.SetSelectedSensorPosition(0, 0, 10);
  LFront.SetSelectedSensorPosition(0, 0, 10);

  RFront.ConfigPeakOutputForward(1.0, 10);
	RFront.ConfigPeakOutputReverse(-1.0, 10);	//Sets RFront to power range -1.0 to 1.0
  RFront.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
  RFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
	RFront.Config_kP(0, 5.25, 10);
  RFront.Config_kI(0, 0, 10);
  RFront.Config_kD(0, 0, 10); //PID values

  LFront.ConfigPeakOutputForward(1.0, 10);
	LFront.ConfigPeakOutputReverse(-1.0, 10);	//Sets LFront to power range -1.0 to 1.0
	LFront.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
  LFront.SetNeutralMode(NeutralMode::Brake); //Brakes when power is 0
  setFollowers(); //Sets motors to follow RFront and LFront
  LFront.Config_kP(0, 5.25, 10);
  LFront.Config_kI(0, 0, 10);
  LFront.Config_kD(0, 0, 10);
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

  rollerEjectButton = gamePad1.GetRawAxis(2); //Left Trigger
  rollerIntakeButton = gamePad1.GetRawAxis(3); //Right Trigger

  if (rollerEjectButton > 0.1) //Checks to see if left trigger is pressed
  { //Press Left Trigger to eject cargo, constant power

    rollerTalon.Set(ControlMode::PercentOutput, rollerEjectButton);//Moves the roller forward used for ejecting cargo
  
  }
  
  else if (rollerIntakeButton > 0.1) //Checks to see if right trigger is pressed
  { //Press Right Trigger to intake cargo, controllable power depending on how much you press it

    rollerTalon.Set(ControlMode::PercentOutput, -rollerIntakeButton);//Moves the roller backward used for intake cargo
  
  }
  
  else
  {

    rollerTalon.Set(ControlMode::PercentOutput, 0);//Stopping the Rollers Motors
  
  }

}

void Robot::shootHatch()
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

void Robot::hatchPeriodic()
{
  ejectButton = gamePad1.GetRawButton(5); //Left Bumper
  grabButton = gamePad1.GetRawButton(6); //Right Bumper
  openButton = gamePad1.GetRawButton(7); //Select  
  if (ejectButton) //Checks to see if Left Bumper is pressed
  {

    shootHatch();
  
  } //Left Bumper pressed, L's open and pusher is extended, then retracted

  else if (grabButton) //Checks to see if Right Bumper is pressed
  {

    topFinger.Set(false); //Closes top finger
    bottomFinger.Set(false); //Closes bottom finger
  
  } //Closes both L's if Right Bumper is pressed

  else if (openButton)
  {

    topFinger.Set(true);
    bottomFinger.Set(true);
  
  } //if Select is pressed, L's are opened without the pusher
}
bool Robot::inRange(int targetDegrees, double currentDegrees, double errorMargin)
{
  //Checks to see if the angle is in the range
  //Error creates a radius away from the target, creating a range for the angle to reach.
  //target - error is the start of the range, target + error is the end of the range
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
{ //Pivots to the range
  float upSpeed = 0.45/4;
  float downSpeed = -0.30/4;
  double amtOff = fabs(targetValue-currentValue);
  double offSetAdjustDown = (amtOff * 0.01);
  double offSetAdjustUp = (amtOff * 0.02);
  
  if (!inRange(targetValue, currentValue, errorValue) 
  && currentValue < targetValue) //Checks if it's past the angle
  {
    pivotBrake.Set(frc::DoubleSolenoid::Value::kForward); 
    pivotTalon.Set(ControlMode::PercentOutput, downSpeed - offSetAdjustDown); 
    //std::cout << "going down: " << downSpeed - offSetAdjustDown << std::endl;

    //Pivots downward
  }
  else if (!inRange(targetValue, currentValue, errorValue) 
  && currentValue > targetValue)
  {
    pivotBrake.Set(frc::DoubleSolenoid::Value::kForward);
    pivotTalon.Set(ControlMode::PercentOutput, upSpeed + offSetAdjustUp);
    //std:: cout << "going up: "<< upSpeed - offSetAdjustUp << std::endl;
    //Pivots upward
  }

  else
  {
    pivotBrake.Set(frc::DoubleSolenoid::Value::kReverse);
    pivotTalon.Set(ControlMode::PercentOutput, 0);
    pivotBrakeMode = true;
    //std::cout << "stopped" << std::endl;
  }

}
void Robot::pivotPeriodic()
{

  //The larger the number is, the further out the pivot is and vice-versa
  //std::cout << "Position: " << pivotPosition << std::endl;
  frc::SmartDashboard::PutNumber("Position", pivotPosition);
  pivotDownButton = gamePad1.GetRawButtonPressed(1); //A
  pivotUpButton = gamePad1.GetRawButtonPressed(2); //B
  double angleRadians = atan2(pivotAccel.GetX(), pivotAccel.GetY()); //Grabs angle in radians
  double angleDegrees = angleRadians * (180/M_PI); //Converts angle to degrees
  //std::cout << "Degrees: " << angleDegrees << std::endl;
  frc::SmartDashboard::PutNumber("Brake mode", pivotBrakeMode);
 


  if (pivotUpButton && pivotPosition != 1 /*&& ((inRange(30, angleDegrees, 10) && pivotPosition == 2) || 
  (inRange(54, angleDegrees, 15) && pivotPosition == 3) || (inRange(137, angleDegrees, 10) && pivotPosition == 4))*/)
  { 

    --pivotPosition; //Moves one position up
    pivotBrakeMode = false;
  }

  else if (pivotDownButton && pivotPosition != 4 /*&& ((inRange(30, angleDegrees, 10) && pivotPosition == 2) || 
  (inRange(54, angleDegrees, 15) && pivotPosition == 3) || (inRange(0, angleDegrees, 5) && pivotPosition == 1))*/)
  {

    ++pivotPosition; //Moves one position down
    pivotBrakeMode = false;
  }
  if(gamePad1.GetRawButton(1) || gamePad1.GetRawButton(2))
  {
    if (pivotPosition == 1 && pivotBrakeMode == false) //Goes to starting position
    {

      goToRange(0, angleDegrees, 2);

    }
    else if (pivotPosition == 2 && pivotBrakeMode == false) //Goes to 30 degrees past starting position
    {

      goToRange(30, angleDegrees, 2);

    }
    else if (pivotPosition == 3 && pivotBrakeMode == false) //Goes to 54 degrees past starting position
    {

      goToRange(54, angleDegrees, 3);
    
    }
    else if (pivotPosition == 4 && pivotBrakeMode == false) //Goes to 137 degrees past starting position
    {

      goToRange(130, angleDegrees, 3);
    
    }

    // std::cout << "angle degrees: " << angleDegrees << std::endl;
    // std::cout << "pivot position: "<< pivotPosition << std::endl;

  }
  else
  {
    pivotBrake.Set(frc::DoubleSolenoid::Value::kReverse);
    pivotTalon.Set(ControlMode::PercentOutput, 0);

    //std::cout << angleDegrees << std::endl;
  }
}

void Robot::pivotTest()
{
  pivotUpButton = gamePad1.GetRawButton(2);
  pivotDownButton = gamePad1.GetRawButton(1);
  if(pivotUpButton)
  {
      pivotBrake.Set(frc::DoubleSolenoid::Value::kForward);
      pivotTalon.Set(ControlMode::PercentOutput, 0.5);
  }
  else if(pivotDownButton)
  {
      pivotBrake.Set(frc::DoubleSolenoid::Value::kForward);
      pivotTalon.Set(ControlMode::PercentOutput, -0.5);
  }
  
  else
  {
    pivotBrake.Set(frc::DoubleSolenoid::Value::kReverse);
    pivotTalon.Set(ControlMode::PercentOutput, 0);
  }
  
}
/* void Robot::habPeriodic() // hab mechanism code
{
    double HABPOWER = 0.2; // constant for motor power
    if (gamePad1.GetRawButton(4)) 
    {
      habTalon.Set(ControlMode::PercentOutput, HABPOWER); // lift the robot until hits the limit swtich
    }
    else if (gamePad1.GetRawButton(3))
    {
      habTalon.Set(ControlMode::PercentOutput, -HABPOWER); // lowers the lift
    }
    else 
    {
      habTalon.Set(ControlMode::PercentOutput, 0); // if nothing else is pressed, stops the lift
    }
} */
void Robot::drivePeriodic()
{

  double leftJoystickPower = -joystickL.GetY();								//Gets y value of left joystick
	double rightJoystickPower = joystickR.GetY();	  						//Gets y value of right joystick
	//std::cout << "Right: " << rightJoystickPower << std::endl;
  //std::cout << "Left: " << leftJoystickPower << std::endl;
  if (joystickL.GetRawButton(1) || joystickR.GetRawButton(1)) //Slowmode if triggers on either joysticks are pressed
	{

		rightJoystickPower *= 0.7;
		leftJoystickPower *= 0.7; //70% power for slowmode
	
  }
  if (rightJoystickPower > 0.1 || rightJoystickPower < -0.1)
  {
	  RFront.Set(ControlMode::PercentOutput, rightJoystickPower); //Sets power to y axis of joysticks
  }
  else
  {

    RFront.Set(ControlMode::PercentOutput, 0);
  
  }
  if (leftJoystickPower > 0.1 || leftJoystickPower < -0.1)
  {

    LFront.Set(ControlMode::PercentOutput, leftJoystickPower);
  }
  else
  {
    LFront.Set(ControlMode::PercentOutput, 0);
  }
}



void Robot::cameraAlign()
{

  auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("limelight");

	inst.GetTable("limelight")->PutNumber("pipeline", 0);//changes camera mode for light sensor

	inst.GetTable("limelight")->PutNumber("ledMode", 0);//Turns camera light on

	double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
	std::cout << targetOffsetAngle_Horizontal << std::endl;
	double offsetAdjust = fabs(targetOffsetAngle_Horizontal * 0.03); //changes how much the alignment adjusts by 0.02
	if (targetOffsetAngle_Horizontal < 0) //turn left
	{
	
  	RFront.Set(ControlMode::PercentOutput, -0.4 - offsetAdjust); // increases power of opposite side instead of decreasing power
		LFront.Set(ControlMode::PercentOutput, 0.4 - offsetAdjust);
	
  } 
	else if (targetOffsetAngle_Horizontal > 0) //turn right
	{

		RFront.Set(ControlMode::PercentOutput, -0.4 + offsetAdjust);
		LFront.Set(ControlMode::PercentOutput, 0.4 + offsetAdjust); // increases power of opposite side instead of decreasing power
	
  }
	else
	{

		RFront.Set(ControlMode::PercentOutput, -0.4);
		LFront.Set(ControlMode::PercentOutput, 0.4); // 0.3

  }

}


void Robot::cameraPeriodicHatch()
{

  cameraAlign(); 
  if (timer.Get() > 1 && RFront.GetSelectedSensorVelocity() == 0 && LFront.GetSelectedSensorVelocity() == 0)
	  { //Checks if velocity is 0
      //Timer needed because Robot starts with 0 velocity
      
		  shootHatch();
	
  } //Opens L's, ejects pusher, then retracts it

}

void Robot::cameraPeriodicCargo()
{

  cameraAlign();
  double angleRadians = atan2(pivotAccel.GetX(), pivotAccel.GetY()); //Grabs angle in radians
  double angleDegrees = angleRadians * (180/M_PI); //Converts angle to degrees
  if (timer.Get() > 1 && RFront.GetSelectedSensorVelocity() == 0 && LFront.GetSelectedSensorVelocity() == 0)
	{

    if (TICKSTOTRAVEL - 19 < -RFront.GetSelectedSensorPosition() && TICKSTOTRAVEL + 19 > -RFront.GetSelectedSensorPosition())
    {

      RFront.Set(ControlMode::Position, -TICKSTOTRAVEL);
      LFront.Set(ControlMode::Position, TICKSTOTRAVEL);
    //If statment to check if RFront is in the range, 19 ticks is about 1/4th of an inch
    
    }
    else
    {

      RFront.Set(ControlMode::PercentOutput, 0);
      LFront.Set(ControlMode::PercentOutput, 0);
    
    }
    //Moving back 7 1/2 inches
    goToRange(-30, angleDegrees, 3.5);
    rollerTalon.Set(ControlMode::PercentOutput, EJECTROLLERPOWER);//Moves the roller forward used for ejecting cargo
  
  } //Aligns, moves right up next to cargo ship, backs up and drops pivot.

}

// void Robot::emergencyPeriodic()
// { //In case of packet loss

//   while (emergencyStop.Get() == 1)
//   {

//     std::cout << "Stopped" << std::endl; //Prints continuously
//     frc::Wait(0.1);
//     RFront.Set(ControlMode::PercentOutput, 0); //Stops all right motors
//     LFront.Set(ControlMode::PercentOutput, 0); //Stops all left motors
//     compressor.SetClosedLoopControl(false); //Closes compressor
//     rollerTalon.Set(ControlMode::PercentOutput, 0); //Stops the roller motor
//     pivotTalon.Set(ControlMode::PercentOutput, 0); //Stops the pivot motor
//     timer.Stop(); //Stops the timer
  
//   }//DO NOT DISABLE WHILE EMERGENCY STOP IS TRUE

// }




void Robot::TeleopPeriodic()
{

  auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("limelight");
  double targetOffset_Skew = table->GetNumber("ts", 0.0);
  //std::cout << targetOffset_Skew << std::endl;

  
  if (joystickR.GetRawButton(4))
  {

    cameraPeriodicHatch(); //Links back to alignment for hatch
  
  }
  else if (joystickR.GetRawButton(3))
  {

    cameraPeriodicCargo(); //Links back to alignment for cargo
  
  }
  else if (joystickR.GetRawButtonPressed(2))
  {

    timer.Reset(); //Resets so that timer only starts during camera alignment

    if (cameraMode == 1)
    {
      cameraMode = 0;
    }

    else if (cameraMode == 0)
    {
      cameraMode = 1;
    }
  }
  else
  {

    timer.Reset(); //Resets so that timer only starts during camera alignment

    rollerPeriodic(); //Links back to roller code

    hatchPeriodic(); //Links back to solenoid fingers and pusher code for hatch

    pivotPeriodic(); //Links back to position code
    //PivotTest();

    drivePeriodic(); //Links back to drive code and slowmode
    
    inst.GetTable("limelight")->PutNumber("pipeline", cameraMode);//changes camera mode for light sensor
    //Driver mode 1 means that LED should be off

		inst.GetTable("limelight")->PutNumber("ledMode", cameraMode);//Turns camera light on/off
    
    //habPeriodic();
    //std::cout << "RFront: " << RFront.GetSelectedSensorPosition() << std::endl;

    //std::cout << "LFront: " << LFront.GetSelectedSensorPosition() << std::endl;

    if (joystickL.GetRawButtonPressed(5) || joystickR.GetRawButtonPressed(5))
    {
      pivotBrake.Set(frc::DoubleSolenoid::Value::kForward); //Forward is unbraked
    }   

    if (joystickL.GetRawButtonPressed(6) || joystickR.GetRawButtonPressed(6))
    {
      pivotBrake.Set(frc::DoubleSolenoid::Value::kReverse); //Reverse is braked
    }


  
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
