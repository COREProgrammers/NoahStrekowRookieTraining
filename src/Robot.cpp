/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
//Vars
/*bool m_AButton = false;
frc::DoubleSolenoid::Value m_desiredGear = frc::DoubleSolenoid::Value::kForward;*/


Robot::Robot(): {}
{}

void Robot::robotInit() {
  
}

void Robot::teleopInit() {}

void Robot::teleop() {
  
  }
}

void Robot::testInit() {}

void Robot::test() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

//hello