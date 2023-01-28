// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrainSub extends SubsystemBase {

  private final WPI_TalonFX m_leftF = new WPI_TalonFX(Constants.driveTrainIDs.leftFID);
  private final WPI_TalonFX m_leftB = new WPI_TalonFX(Constants.driveTrainIDs.leftBID);
  private final MotorControllerGroup left = new MotorControllerGroup(m_leftF, m_leftB);

  private final WPI_TalonFX m_rightF = new WPI_TalonFX(Constants.driveTrainIDs.rightBID);
  private final WPI_TalonFX m_rightB = new WPI_TalonFX(Constants.driveTrainIDs.rightBID);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightF, m_rightB);

  private final DifferentialDrive drive = new DifferentialDrive(left, right);
  
  /** Creates a new ExampleSubsystem. */
  public driveTrainSub() {

    right.setInverted(true);

  }

  public void tankDrive(double leftSpeed, double rightSpeed){

    drive.tankDrive(leftSpeed, rightSpeed);

  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
