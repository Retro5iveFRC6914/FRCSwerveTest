// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.driveTrainSub;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class defaultDrive extends CommandBase {

  private final driveTrainSub m_drive;
  private Joystick leftStick = new Joystick(Constants.joystickIDs.leftStickID);
  private Joystick rightStick = new Joystick(Constants.joystickIDs.rightStickID);
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public defaultDrive(driveTrainSub drive, Joystick lftStick, Joystick rghtStick) {
    addRequirements(drive);
    m_drive = drive;
    leftStick = lftStick;
    rightStick = rghtStick;
  }
  @Override
  public void execute(){
    double inputLeft = leftStick.getY();
    double inputRight = rightStick.getY();

    m_drive.tankDrive(inputLeft, inputRight);
  }

}
