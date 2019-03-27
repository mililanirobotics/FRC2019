/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/WPILib.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot 
{

 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void pivotInit();
  void pivotTest();
  void pivotPeriodic();
  void rollerInit();
  void rollerPeriodic();
  void hatchInit();
  void hatchPeriodic();
  bool inRange(int targetDegrees, double currentDegrees, double errorMargin);
  void goToRange(int targetValue, double currentValue, double errorValue);
  void setFollowers();
  void driveInit();
  void drivePeriodic();
  void cameraAlign();
  void cameraPeriodicHatch();
  void cameraPeriodicCargo();
  //void emergencyPeriodic();
  void shootHatch();
  //void habPeriodic();
  
  //Talons
  TalonSRX LBack{10};
  TalonSRX LMiddle{14};
  TalonSRX LFront{12};
  TalonSRX RBack{11};
  TalonSRX RMiddle{15};
  TalonSRX RFront{13};
  TalonSRX rollerTalon{17}; //Intake/Output roller on cargo payload
  TalonSRX pivotTalon{16}; //talon that moves pivot


  //Pneumatics
  frc::Compressor compressor{0};
  frc::Solenoid topFinger{1};
  frc::Solenoid bottomFinger{2};
  frc::Solenoid rightPusher{0};
  frc::Solenoid leftPusher{3};
  frc::DoubleSolenoid pivotBrake{4, 5}; 

  //Controller Settings
  frc::Joystick joystickL{0}; //Evan's left controller
  frc::Joystick joystickR{1}; //Evan's right controller
  frc::Joystick gamePad1{2}; //Tariq's controller

  //Accelerometer
  frc::ADXL345_I2C pivotAccel{frc::I2C::Port::kOnboard};

  //Timer
  frc::Timer timer{};

  double rollerEjectButton; //Left Trigger
  double rollerIntakeButton; //Right Trigger
  bool ejectButton; //Left Bumper
  bool grabButton; //Right Bumper
  bool openButton; //Select
  int pivotPosition = 1; //Starting position
  bool pivotUpButton;
  bool pivotDownButton;
  bool pivotBrakeMode = false;
  int cameraMode = 1;
  const double WHEELCIRCUMFERENCE = 6 * M_PI;
  const double DISTANCEFROMSHIP = 7.5;
  const double TICKSPERROT = 1440; //Tested value
  const double TICKSTOTRAVEL = (DISTANCEFROMSHIP/WHEELCIRCUMFERENCE) * TICKSPERROT; //Ticks traveled for 7 1/2 inches
};
