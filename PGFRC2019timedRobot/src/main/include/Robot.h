/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "ctre/Phoenix.h"
#include "frc/WPILib.h"
#include "frc/Compressor.h"

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void setFollowers();

 private:
  const float WHEEL_CIRCUMFERENCE = 18.84955592153876;
  const double GEAR_RATIO = 1;
  const double TICKS_PER_ROTATION = 1;
  const int OFF_SET_DISTANCE = 400;
  //Talons
  TalonSRX LBack{10};
  TalonSRX LMiddle{14};
  TalonSRX LFront{12};
  TalonSRX RBack{11};
  TalonSRX RMiddle{15};
  TalonSRX RFront{13};
  TalonSRX pivotTalon{16};
  TalonSRX rollerTalon{17};
  
  //Limit switches
  frc::DigitalInput limitSwitchOne{0};
  frc::DigitalInput limitSwitchTwo{1};
  
  //Solenoids
  frc::Solenoid solenoidOne{4, 5};
  frc::Solenoid solenoidTwo{6, 7};
  frc::Solenoid solenoidThree{0, 1};
  frc::Solenoid solenoidFour{2, 3};
  frc::Compressor compressor{2};
  
  //Controllers
  frc::Joystick joystickL{0};
  frc::Joystick joystickR{1};
  frc::Joystick gamePad1{2};

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
