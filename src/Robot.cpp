/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

//Vars
/*bool m_AButton = false;
frc::DoubleSolenoid::Value m_desiredGear = frc::DoubleSolenoid::Value::kForward;*/


Robot::Robot(): compressor(1),
                m_leftFrontMotor(LEFT_FRONT_MOTOR_PORT),
                m_rightFrontMotor(RIGHT_FRONT_MOTOR_PORT),
                m_leftBackMotor(LEFT_BACK_MOTOR_PORT),
                m_rightBackMotor(RIGHT_BACK_MOTOR_PORT),
                m_engageLeftMotorSolenoid(ENGAGE_LEFT_MOTOR_SOLENOID_PCM_PORT, ENGAGE_LEFT_MOTOR_SOLENOID_OPEN_PORT, ENGAGE_LEFT_MOTOR_SOLENOID_CLOSE_PORT),
                m_engageRightMotorSolenoid(ENGAGE_RIGHT_MOTOR_SOLENOID_PCM_PORT, ENGAGE_RIGHT_MOTOR_SOLENOID_OPEN_PORT, ENGAGE_RIGHT_MOTOR_SOLENOID_CLOSE_PORT)
{}

void Robot::robotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  m_solenoidEngaged = false;
  m_leftFrontMotor.Set(ControlMode::PercentOutput, 0);
  m_rightFrontMotor.Set(ControlMode::PercentOutput, 0);
  m_leftBackMotor.Set(ControlMode::PercentOutput, 0);
  m_rightBackMotor.Set(ControlMode::PercentOutput, 0);
  operatorJoystick->RegisterButton(CORE::COREJoystick::A_BUTTON);
  operatorJoystick->RegisterAxis(CORE::COREJoystick::RIGHT_STICK_Y);
}

void Robot::teleopInit() {}

void Robot::teleop() {
  //on an A button press, change engaged
  operatorJoystick->GetRisingEdge(CORE::COREJoystick::A_BUTTON); {
      m_solenoidEngaged = !m_solenoidEngaged;
  }
  
  //if Solenoid is engaged, move motors
  double mag = operatorJoystick->GetAxis(CORE::COREJoystick::RIGHT_STICK_Y);
  if (m_solenoidEngaged) {
    m_leftFrontMotor.Set(ControlMode::PercentOutput, mag * 0.9);
    m_rightFrontMotor.Set(ControlMode::PercentOutput, mag * -0.9);
    m_leftBackMotor.Set(ControlMode::PercentOutput, mag * 0.9);
    m_rightBackMotor.Set(ControlMode::PercentOutput, mag * -0.9);
  } else {
    m_leftFrontMotor.Set(ControlMode::PercentOutput, 0);
    m_rightFrontMotor.Set(ControlMode::PercentOutput, 0);
    m_leftBackMotor.Set(ControlMode::PercentOutput, 0);
    m_rightBackMotor.Set(ControlMode::PercentOutput, 0);
  }
}

void Robot::testInit() {}

void Robot::test() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
