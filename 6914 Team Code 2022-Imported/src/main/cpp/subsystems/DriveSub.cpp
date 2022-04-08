// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <Constants.h>

class DriveSub : public frc::TimedRobot {

ctre::phoenix::motorcontrol::can::WPI_TalonSRX leftf{1};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX leftb{5};
frc::SpeedControllerGroup leftDrive{leftf, leftb};

ctre::phoenix::motorcontrol::can::WPI_TalonSRX rightf{2};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX rightb{4};
frc::SpeedControllerGroup rightDrive{rightf, rightb};

frc::DifferentialDrive arcadeDrive{leftDrive, rightDrive};

frc::Joystick rightStick{0};

public:
void Drivetrain() {

  arcadeDrive.TankDrive(rightStick.GetRawAxis(1), rightStick.GetRawAxis(3));

}

};