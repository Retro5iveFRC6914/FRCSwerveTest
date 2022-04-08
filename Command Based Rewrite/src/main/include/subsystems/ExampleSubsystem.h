// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Joystick.h> 
#include "Constants.h"
#include <frc/GenericHID.h>

using namespace oiConstants;

class ExampleSubsystem : public frc2::SubsystemBase {
 public:
  ExampleSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
// Components (e.g. motor controllers and sensors) should generally be
// declared private and exposed only through public methods.
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_leftF;
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_leftB;
frc::MotorControllerGroup left{m_leftF, m_leftB};

ctre::phoenix::motorcontrol::can::WPI_TalonFX m_rightF;
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_rightB;
frc::MotorControllerGroup right{m_rightF, m_rightB};

frc::DifferentialDrive tankDrive{left, right};    //make left side and right side into one drive - tank drive
};
