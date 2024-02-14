// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RotateForward extends CommandBase {
 
  private final Intake intakeSubsystem;
  private final double speed;

  public RotateForward(intakeSubsystem intakeSubsystem, double speed) {
    this.intakeSubsystem intakeSubsystem = intakeSubsystem;
    this.speed = speed;
    addRequirements(Intake.subsystem);

  /** Creates a new RotateForward. */

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    public RotateForward() {
      System.out.println("Intake.subsystem started");
  }
}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotor(0);
    System.out.println("RotateForward ended");

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
