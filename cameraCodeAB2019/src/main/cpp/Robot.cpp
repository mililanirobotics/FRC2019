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
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>


Robot::Robot():
	LBack(10),
	LMiddle(14),
	LFront(12),
	RBack(11),
	RMiddle(15),
	RFront(13),
	solenoidOne(2, 3),
	compressor(2),
	stickOne(0),
	stickTwo(1),
	gamePad1(2)
	{

		// Note SmartDashboard is not initialized here, wait until
		// RobotInit to make SmartDashboard calls
		m_robotDrive.SetExpiration(0.1);
	}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}
void Robot::setFollowers(){
	LBack.Set(ControlMode::Follower, 12);
	LMiddle.Set(ControlMode::Follower, 12);
	RBack.Set(ControlMode::Follower, 13);
	RMiddle.Set(ControlMode::Follower, 13);
}

void Robot::moveLFront(double distanceInInches, bool forward){
		float rotationsOfWheel = distanceInInches / WHEEL_CIRCUMFERENCE;
		float shaftRotations = rotationsOfWheel * GEAR_RATIO;
		float ticksMoved = shaftRotations * TICKS_PER_ROTATION;
		distanceInInches += OFF_SET_DISTANCE;
		LFront.SetSelectedSensorPosition(0, 0, 10);
		std::cout << LBack.GetSelectedSensorPosition(0) << std::endl;
		//While loop to give time for the robot set position to 0
		while (IsAutonomous() && IsEnabled() && LFront.GetSelectedSensorPosition(0) != 0){
			std::cout << LFront.GetSelectedSensorPosition(0) << std::endl;
			frc::Wait(0.1);
		}
		//Wait(0.1);
		if (forward){
			LFront.Set(ControlMode::Position, ticksMoved);
		}
		else{
			LFront.Set(ControlMode::Position, -ticksMoved);
		}
		if (forward){
			while (IsAutonomous() && IsEnabled() && LFront.GetSelectedSensorPosition(0) <= distanceInInches - OFF_SET_DISTANCE - 50){
				std::cout << LFront.GetSelectedSensorPosition(0) << std::endl;
				std::cout << "forwar" << std::endl;
				frc::Wait(1);
			}
		}
		else{
			while (IsAutonomous() && IsEnabled() && LFront.GetSelectedSensorPosition(0) >= -distanceInInches + OFF_SET_DISTANCE + 50){
				std::cout << LFront.GetSelectedSensorPosition(0) << std::endl;
				std::cout << "back" << std::endl;
				frc::Wait(1);
			}
		}
		LFront.Set(ControlMode::PercentOutput, 0);
		std::cout << "Turn\n";
		std::cout << LFront.GetSelectedSensorPosition(0) << std::endl;
		LFront.SetSelectedSensorPosition(0, 0, 10);
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
void Robot::Autonomous() 
{
}

/**
 * Runs the motors with arcade steering.
 */
void Robot::OperatorControl() {
  RFront.ConfigPeakOutputForward(1.0, 10);
	RFront.ConfigPeakOutputReverse(-1.0, 10);
	RFront.SetNeutralMode(NeutralMode::Brake);
	RFront.SetSensorPhase(false);
	LFront.ConfigPeakOutputForward(1.0, 10);
	LFront.ConfigPeakOutputReverse(-1.0, 10);
	LFront.SetNeutralMode(NeutralMode::Brake);
	LFront.SetSensorPhase(false);

  	auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("limelight");

	double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);

	double Lstick1;
	double Rstick1;
	m_robotDrive.SetSafetyEnabled(false);
	compressor.SetClosedLoopControl(true);
	setFollowers();
	while (IsOperatorControl() && IsEnabled()) {
		//Drive with arcade style (use right stick)
		//m_robotDrive.ArcadeDrive(
				//-m_stick.GetY(), m_stick.GetX());
		Lstick1 = -stickOne.GetY();
		Rstick1 = stickTwo.GetY();
		if (stickOne.GetRawButton(1) || stickTwo.GetRawButton(1)){
			Rstick1 *= 0.7;
			Lstick1 *= 0.7;
		}
		RFront.Set(ControlMode::PercentOutput, Rstick1);
		LFront.Set(ControlMode::PercentOutput, Lstick1);
		//std::cout << Rstick1 << std::endl;
		//std::cout << Lstick1 << std::endl;
		if (gamePad1.GetRawButton(5)){
			solenoidOne.Set(frc::DoubleSolenoid::Value::kForward);
		}
		if (gamePad1.GetRawButton(6)){
			solenoidOne.Set(frc::DoubleSolenoid::Value::kReverse);
		}
    if (gamePad1.GetRawButton(2)){
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
		//The motors will be updated every 5ms
		frc::Wait(0.005);
	}
}

/**
 * Runs during test mode
 */
void Robot::Test() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
