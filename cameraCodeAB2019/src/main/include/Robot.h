/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include "ctre/Phoenix.h"

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SampleRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use TimedRobot or Command-Based
 * instead if you're new.
 */
class Robot : public frc::SampleRobot {
 public:
  Robot();

  void RobotInit() override;
  void Autonomous() override;
  void OperatorControl() override;
  void Test() override;
  void setFollowers();
  void moveLFront(double distanceInInches, bool forward);

 private:

 	const float WHEEL_CIRCUMFERENCE = 18.84955592153876;
	const double GEAR_RATIO = 1;
	const double TICKS_PER_ROTATION = 1;
	const int OFF_SET_DISTANCE = 400;
	TalonSRX RFront;
  TalonSRX RMiddle;
	TalonSRX RBack;
	TalonSRX LFront;
  TalonSRX LMiddle;
	TalonSRX LBack;
  frc::DoubleSolenoid solenoidOne;
  frc::Compressor compressor;
	frc::Joystick stickOne{0};
  
  frc::Joystick stickTwo{1};

  frc::Joystick gamePad1{2};

  // Robot drive system
  frc::PWMVictorSPX m_leftMotor{0};
  frc::PWMVictorSPX m_rightMotor{1};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

  //frc::Joystick m_stick{0};

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
};