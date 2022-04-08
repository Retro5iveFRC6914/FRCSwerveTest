// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import 
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrainSub extends SubsystemBase {

  WPI_TalonFX m_leftF = new WPI_TalonFX(1);
  WPI_TalonFX m_leftB = new WPI_TalonFX(3);
  MotorControllerGroup left = new MotorControllerGroup(m_leftF, m_leftB);

  WPI_TalonFX m_rightF = new WPI_TalonFX(2);
  WPI_TalonFX m_rightB = new WPI_TalonFX(4);
  MotorControllerGroup right = new MotorControllerGroup(m_rightF, m_rightB);

  DifferentialDrive tankDrive = new DifferentialDrive(left, right);
  
  /** Creates a new ExampleSubsystem. */
  public driveTrainSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
