/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
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
	pivotTalon(16),
	rollerTalon(17),
	limitSwitchOne(0),
	limitSwitchTwo(1),
	solenoidOne(4, 5),
	solenoidTwo(6, 7),
	solenoidThree(0,1),
	solenoidFour(2, 3),
	compressor(0),
	stickOne(0), //Joystick one
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
void Robot::Autonomous() {

		//std::string autoSelected = m_chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString(
		// "Auto Selector", kAutoNameDefault);
		//std::cout << "Auto selected: " << autoSelected << std::endl;

		// MotorSafety improves safety when motors are updated in loops
		// but is disabled here because motor updates are not looped in
		// this autonomous mode.
		m_robotDrive.SetSafetyEnabled(false);
		setFollowers();
		/*if (autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
			std::cout << "Running custom Autonomous" << std::endl;

			// Spin at half speed for two seconds
			m_robotDrive.ArcadeDrive(0.0, 0.5);
			frc::Wait(2.0);

			// Stop robot
			m_robotDrive.ArcadeDrive(0.0, 0.0);
		} else {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;

			// Drive forwards at half speed for two seconds
			m_robotDrive.ArcadeDrive(-0.5, 0.0);
			frc::Wait(2.0);

			// Stop robot
			m_robotDrive.ArcadeDrive(0.0, 0.0);
		}*/
		std::cout << "a" << std::endl;
		RFront.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		RFront.SetSelectedSensorPosition(0, 0, 10);
		RFront.ConfigPeakOutputForward(0.3, 10);
		RFront.ConfigPeakOutputReverse(-0.3, 10);
		RFront.SetNeutralMode(NeutralMode::Brake);
		LBack.Set(ControlMode::Follower, 12);
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
	double ROLLERPOWER = 0.3;
	double leftJoystickPower;
	double rightJoystickPower;
	m_robotDrive.SetSafetyEnabled(false);
	compressor.SetClosedLoopControl(true);
	setFollowers();

	while (IsOperatorControl() && IsEnabled()) {
		//Drive with arcade style (use right stick)
		//m_robotDrive.ArcadeDrive(
				//-m_stick.GetY(), m_stick.GetX());
		leftJoystickPower = -stickOne.GetY();
		rightJoystickPower = stickTwo.GetY();
		if (stickOne.GetRawButton(1) || stickTwo.GetRawButton(1)){
			rightJoystickPower *= 0.7;
			leftJoystickPower *= 0.7;
		}
		RFront.Set(ControlMode::PercentOutput, rightJoystickPower);
		LFront.Set(ControlMode::PercentOutput, leftJoystickPower);
		if (gamePad1.GetRawButton(5)){
			solenoidOne.Set(frc::DoubleSolenoid::Value::kForward);
			solenoidTwo.Set(frc::DoubleSolenoid::Value::kForward);
			frc::Wait(0.02);
			solenoidThree.Set(frc::DoubleSolenoid::Value::kForward);
			solenoidFour.Set(frc::DoubleSolenoid::Value::kForward);

		}

		if (gamePad1.GetRawButton(6)){
			solenoidOne.Set(frc::DoubleSolenoid::Value::kReverse);
			solenoidTwo.Set(frc::DoubleSolenoid::Value::kReverse);
		}

		if (gamePad1.GetRawButton(2) && limitSwitchOne.Get() == 0){
			pivotTalon.Set(ControlMode::PercentOutput, 0.3);
		}

		else if (gamePad1.GetRawButton(3) && limitSwitchTwo.Get() == 0){
			pivotTalon.Set(ControlMode::PercentOutput, -0.3);
		}

		else {
			pivotTalon.Set(ControlMode::PercentOutput, 0);
		}

			
		if (gamePad1.GetRawButton(3)){
			solenoidThree.Set(frc::DoubleSolenoid::Value::kForward);
			solenoidFour.Set(frc::DoubleSolenoid::Value::kForward);
		}
		if (gamePad1.GetRawButton(2)){
			solenoidThree.Set(frc::DoubleSolenoid::Value::kReverse);
			solenoidFour.Set(frc::DoubleSolenoid::Value::kReverse);
		}

		if (gamePad1.GetRawButton(1)){
			rollerTalon.Set(ControlMode::PercentOutput, ROLLERPOWER);
		}
		
		else if (gamePad1.GetRawButton(4)){
			rollerTalon.Set(ControlMode::PercentOutput, -ROLLERPOWER);
		}

		else{
			rollerTalon.Set(ControlMode::PercentOutput, 0);
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
