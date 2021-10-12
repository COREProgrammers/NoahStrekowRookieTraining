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

  
};

//hello