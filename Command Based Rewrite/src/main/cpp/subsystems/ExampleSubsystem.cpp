// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExampleSubsystem.h"

ExampleSubsystem::ExampleSubsystem() 
 :m_leftF{fLeftID},
 m_leftB{bLeftID},
 m_rightF{fRightID},
 m_rightB{bRightID} {

right.SetInverted(true);

 }


void ExampleSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ExampleSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void ExampleSubsystem::TankDrive(double fwd, double rot) {
  
}
