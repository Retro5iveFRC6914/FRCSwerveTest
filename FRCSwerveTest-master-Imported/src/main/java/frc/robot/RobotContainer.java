// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.BlindFire;
import frc.robot.commands.Warning;
import frc.robot.commands.HoldArm;
import frc.robot.commands.RunArmClosedLoop;
import frc.robot.commands.HoldClimber;
import frc.robot.commands.HomeClimber;
import frc.robot.commands.RunClimberManual;

import frc.robot.commands.Feed;
import frc.robot.commands.HoldIntake;
import frc.robot.commands.IntakeNoteAutomatic;
import frc.robot.commands.RunIntakeOpenLoop;
import frc.robot.commands.AccelerateShooter;
import frc.robot.commands.RunShooterAtVelocity;
import frc.robot.commands.ShootNote;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final Swerve m_robotDrive = new Swerve();
  public final Arm m_arm = new Arm(ArmConstants.kLeftID, ArmConstants.kRightID);
  public final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kSensorDIOPort);
  public final Shooter m_shooter = new Shooter(ShooterConstants.kTopID, ShooterConstants.kBottomID);
  public final Climber m_portClimber = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO);
  public final Climber m_starboardClimber = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO);



  // The driver's controller
  CommandXboxController operatorXboxController = new CommandXboxController(0);
  CommandJoystick driverXboxController = new CommandJoystick(3);


  // Command Groups
  ParallelCommandGroup feedAndShootSubwoofer = new ParallelCommandGroup(
    new ShootNote(m_shooter, ShooterConstants.kSubwooferSpeed),
    new Feed(m_intake)
  );
  ParallelCommandGroup feedAndShootPodium = new ParallelCommandGroup(
    new ShootNote(m_shooter, ShooterConstants.k3mSpeed),
    new Feed(m_intake)
  );
  SequentialCommandGroup shootSubwoofer = new SequentialCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos),
    new AccelerateShooter(m_shooter, ShooterConstants.kSubwooferSpeed),
    feedAndShootSubwoofer
  );
  SequentialCommandGroup shootPodium = new SequentialCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
    new AccelerateShooter(m_shooter, ShooterConstants.k3mSpeed),
    feedAndShootPodium
  );
  ParallelCommandGroup intake = new ParallelCommandGroup(
    new IntakeNoteAutomatic(m_intake),
    new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)
  );

  ParallelCommandGroup homeClimbers = new ParallelCommandGroup(
    new HomeClimber(m_portClimber),
    new HomeClimber(m_starboardClimber)
  );

  ParallelCommandGroup forceFeed = new ParallelCommandGroup(
    new RunIntakeOpenLoop(m_intake, IntakeConstants.kReverseSpeed),
    new RunShooterAtVelocity(m_shooter, ShooterConstants.kAmpSpeed)
  );
  ParallelCommandGroup forceReverse = new ParallelCommandGroup(
    new RunIntakeOpenLoop(m_intake, -IntakeConstants.kReverseSpeed),
    new RunShooterAtVelocity(m_shooter, -ShooterConstants.kAmpSpeed)
  );

  SequentialCommandGroup amp = new SequentialCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
    forceFeed
  );

  private final SendableChooser<Command> autoChooser;


    // Configure the button bindings
        /** The container for the robot. Contains subsystems, OI devices, and commands. */
        public RobotContainer () {
          s_Swerve.setDefaultCommand(
              new TeleopSwerve(
                  s_Swerve, 
       () -> -driver.getRawAxis(translationAxis),
                  () -> -driver.getRawAxis(strafeAxis), 
                  () -> -driver.getRawAxis(rotationAxis), 
                  () -> robotCentric.getAsBoolean()


    configureButtonBindings();

    // Register Named Commands
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("shootSubwoofer", shootSubwoofer);
    NamedCommands.registerCommand("shootpodium", shootPodium);
    NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
    NamedCommands.registerCommand("armDown", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));



    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    m_arm, m_intake, m_shooter, m_portClimber, m_starboardClimber, autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(

      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(driverXboxController.getX()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverXboxController.getY()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverXboxController.getTwist()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          true, true), m_robotDrive));
    
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    operatorXboxController.leftStick().or(operatorXboxController.rightStick()).toggleOnTrue(
    new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(operatorXboxController.getLeftY()*OperatorConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(operatorXboxController.getLeftX()*OperatorConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(operatorXboxController.getRightX()*OperatorConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
        true, true), ));

    driverXboxController.povCenter().whileFalse(
        new RapidHeading(
            driverXboxController.getHID().getPOV(),
            -MathUtil.applyDeadband(driverXboxController.getY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverXboxController.getX(), OIConstants.kDriveDeadband),
            ));

    driverXboxController.button(DriverConstants.kTurbo).whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(driverXboxController.getX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverXboxController.getY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverXboxController.getTwist(), OIConstants.kDriveDeadband),
          true, true), ));
    

    operatorXboxController.start().whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(operatorXboxController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(operatorXboxController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(operatorXboxController.getRightX(), OIConstants.kDriveDeadband),
          false, true), ));

    // Subsystem Default Commands
    m_intake.setDefaultCommand(new HoldIntake(m_intake));
    m_arm.setDefaultCommand(new HoldArm(m_arm));
    m_shooter.setDefaultCommand(new RunShooterAtVelocity(m_shooter, ShooterConstants.kIdleSpeed));
    m_portClimber.setDefaultCommand(new HoldClimber(m_portClimber));
    m_starboardClimber.setDefaultCommand(new HoldClimber(m_starboardClimber));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    driverXboxController.button(DriverConstants.kSetX)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            ));

    driverXboxController.button(DriverConstants.kIntake).whileTrue(intake);
    driverXboxController.button(DriverConstants.kHoldArmDown).toggleOnTrue(new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));
    driverXboxController.button(DriverConstants.kAutoIntake).and(driverXboxController.button(DriverConstants.kIntake)).onTrue(
      new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );
    driverXboxController.button(DriverConstants.kIntake).and(driverXboxController.button(DriverConstants.kAutoIntake)).onTrue(
      new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );
    driverXboxController.button(DriverConstants.kSelfDestruct).onTrue(
      new InstantCommand(() -> System.out.println("¡KABOOM!"))
    );

    operatorXboxController.start().whileTrue(new Warning("¡OVERRIDE!"));
    operatorXboxController.start().and(operatorXboxController.povUp()).whileTrue(new RunArmClosedLoop(m_arm, ArmConstants.kManualSpeed));
    operatorXboxController.start().and(operatorXboxController.povDown()).whileTrue(new RunArmClosedLoop(m_arm, -ArmConstants.kManualSpeed));
    operatorXboxController.start().and(operatorXboxController.povRight()).whileTrue(forceFeed);
    operatorXboxController.start().and(operatorrXboxController.povLeft()).whileTrue(forceReverse);

  operatorXboxController.a().whileTrue(new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos));
    operatorXboxController.b().whileTrue(shootSubwoofer);
    operatorXboxController.y().whileTrue(new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos));
    operatorXboxController.x().whileTrue(new IntakeNoteAutomatic(m_intake));
    operatorXboxController.x().whileTrue( new RunShooterAtVelocity(m_shooter, ShooterConstants.kSubwooferSpeed));


    operatorXboxController.back().whileTrue(homeClimbers);
    operatorXboxController.rightBumper().whileTrue(new RunClimberManual(m_starboardClimber, ClimberConstants.kManualSpeed));
    operatorXboxController.rightTrigger().whileTrue(new RunClimberManual(m_portClimber, -ClimberConstants.kManualSpeed));
    operatorXboxController.leftBumper().whileTrue(new RunClimberManual(m_portClimber, ClimberConstants.kManualSpeed));
    operatorXboxController.leftTrigger().whileTrue(new RunClimberManual(m_portClimber, ClimberConstants.kManualSpeed));

  }
    
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}