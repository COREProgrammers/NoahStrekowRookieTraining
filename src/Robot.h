/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <ctre/Phoenix.h>
#include <CORERobotLib.h>
#include <COREFramework/COREScheduler.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
using namespace frc;

//Motor Definitons
#define LEFT_FRONT_MOTOR_PORT 11
#define RIGHT_FRONT_MOTOR_PORT 12
#define LEFT_BACK_MOTOR_PORT 10
#define RIGHT_BACK_MOTOR_PORT 13
//Solenoid Definitions
#define ENGAGE_RIGHT_MOTOR_SOLENOID_PCM_PORT 1
#define ENGAGE_LEFT_MOTOR_SOLENOID_PCM_PORT 1
#define ENGAGE_RIGHT_MOTOR_SOLENOID_OPEN_PORT 2
#define ENGAGE_LEFT_MOTOR_SOLENOID_OPEN_PORT 0
#define ENGAGE_RIGHT_MOTOR_SOLENOID_CLOSE_PORT 3
#define ENGAGE_LEFT_MOTOR_SOLENOID_CLOSE_PORT 1

// #include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public CORERobot {
 public:
  Robot();
  void robotInit() override;
  void teleop() override;
  void teleopInit() override;
  void test() override;
  void testInit() override;


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  TalonSRX m_leftFrontMotor, m_rightFrontMotor, m_leftBackMotor, m_rightBackMotor;
  DoubleSolenoid m_engageLeftMotorSolenoid, m_engageRightMotorSolenoid;
  bool m_solenoidEngaged;
  Compressor compressor;
};
