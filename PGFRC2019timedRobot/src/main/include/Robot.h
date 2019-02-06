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
#include "frc/Solenoid.h"
#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DigitalInput.h>
class Robot : public frc::TimedRobot {
 public:
  //Functions being initialized
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void setFollowers();
  void drivePeriodic();
  void solenoidPeriodic();
  void pivotPeriodic();
  void rollerPeriodic();
  void cameraPeriodic();
  void driveInit();
  void rollerInit();
 private:
  //Constants
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
  //TalonSRX pivotTalon{16};
  //TalonSRX rollerTalon{17};
  
  //Limit switches
  frc::DigitalInput limitSwitchOne{0};
  frc::DigitalInput limitSwitchTwo{1};
  
  //Solenoids
  frc::Solenoid topFinger{0};
  frc::Solenoid bottomFinger{2};
  frc::Solenoid rightPusher{1};
  frc::Solenoid leftPusher{3};
  frc::Compressor compressor{0};
  
  //Controllers
  frc::Joystick joystickL{0};
  frc::Joystick joystickR{1};
  frc::Joystick gamePad1{2};

  //timer
  frc::Timer timer{};
  
  //Emergency Stop
  frc::DigitalInput emergencyStop{9};

  //Emergency Stop
  frc::DigitalInput emergencyStop{9};  

  //Accelerometer
  frc::ADXL345_I2C pivotAccel{frc::I2C::Port::kOnboard};
  
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
