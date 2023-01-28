// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  double shooterTicks = 8000;
  double indexSpeed = 0.7;
  double intakeSpeed = 0.9;

  boolean toggle = false;
  boolean shooterBool;

  WPI_TalonFX m_leftF = new WPI_TalonFX(5);
  WPI_TalonFX m_leftB = new WPI_TalonFX(2);
  MotorControllerGroup left = new MotorControllerGroup(m_leftF, m_leftB);

  WPI_TalonFX m_rightF = new WPI_TalonFX(4);
  WPI_TalonFX m_rightB = new WPI_TalonFX(3);
  MotorControllerGroup right = new MotorControllerGroup(m_rightF, m_rightB);

  DifferentialDrive tankDrive = new DifferentialDrive(left, right);

  WPI_TalonFX shooter = new WPI_TalonFX(7);
  WPI_TalonFX climber = new WPI_TalonFX(9);
  WPI_TalonFX climber2 = new WPI_TalonFX(6);
  WPI_TalonFX index = new WPI_TalonFX(8);
  WPI_TalonFX intake = new WPI_TalonFX(5);

  Joystick leftStick = new Joystick(0);
  Joystick rightStick = new Joystick(1);
  GenericHID mechPad = new GenericHID(2);

  Timer timer = new Timer();
  int kSlotIdx = 0;
  int kPIDLoopIdx = 0;
  int kTimeoutMs = 10;

  public boolean readyToFire;


  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    right.setInverted(true);
    CameraServer.startAutomaticCapture();

    shooter.setInverted(true);
    shooter.configFactoryDefault();

    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, kTimeoutMs);
    shooter.setSensorPhase(true);

    shooter.configNominalOutputForward(0, kTimeoutMs);
    shooter.configNominalOutputReverse(0, kTimeoutMs);
    shooter.configPeakOutputForward(1, kTimeoutMs);
    shooter.configPeakOutputReverse(-1, kTimeoutMs);

    shooter.config_kF(kPIDLoopIdx, 0.5, kTimeoutMs);
    shooter.config_kP(kPIDLoopIdx, 0.5, kTimeoutMs);
    shooter.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    shooter.config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    /*double shooterSpeed = shooter.getSelectedSensorVelocity();

    if(shooterSpeed >= 10000){

      shooterBool = true;

    }else{

      shooterBool = false;

    }

    SmartDashboard.putBoolean("Shooter On?", shooterBool);
    */

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.reset();
    timer.start();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { //autonomous = 15
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;

    } 

        if(Timer.getMatchTime() > 11.0 && Timer.getMatchTime() < 15.0){
          
          shooter.set(TalonFXControlMode.Velocity, 3500);
          tankDrive.tankDrive(-0.3, -0.3);

        }else if(Timer.getMatchTime() > 8.0 && Timer.getMatchTime() < 11.0){

          shooter.set(TalonFXControlMode.Velocity, 3500);
          index.set(-1.0);
          intake.set(1);
          tankDrive.tankDrive(0, 0);

        }else if(Timer.getMatchTime() > 6.0 && Timer.getMatchTime() < 8.0){

          index.set(0);
          shooter.set(0);
          intake.set(0);
          tankDrive.tankDrive(0.6, 0.6);

        }else if(Timer.getMatchTime() == 6.0){

          tankDrive.tankDrive(0, 0);

        }else{

          index.set(0);
          shooter.set(0);
          tankDrive.tankDrive(0, 0);
          intake.set(0);

        }
    

    

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

      tankDrive.tankDrive(rightStick.getRawAxis(1), leftStick.getRawAxis(1));

      SmartDashboard.putNumber("Current t/r", shooter.getSelectedSensorVelocity());
      SmartDashboard.putBoolean("Ready to Fire?", readyToFire);
      double timeNow = Timer.getMatchTime();

      SmartDashboard.putNumber("Match Time Now", timeNow);

      shooterFunction();
      indexFunction();
      intakeFunction();
      climberFunction();
      climber2Function();

      if(shooter.getSelectedSensorVelocity() >= 5000){
        readyToFire = true;
      }
      else{
        readyToFire = false;
      }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void shooterFunction(){
    if(mechPad.getRawButtonPressed(3)){

      if(toggle){
        
        shooter.set(0);
        toggle = false;
      
      }else{

        shooter.set(ControlMode.Velocity, shooterTicks);
        toggle = true;
      }
    }
  }

  public void indexFunction(){

    if(mechPad.getRawButton(8)){

      index.set(indexSpeed);

    }else if(mechPad.getRawButton(6)){

      index.set(-(indexSpeed));

    }else {

      index.set(0);
    }
    
  }

  public void intakeFunction(){

    if(mechPad.getRawButton(7)){

      intake.set(intakeSpeed);

    }else if(mechPad.getRawButton(5)){

      intake.set(-(intakeSpeed));

    }else{

      intake.set(0);

    }
  }

  public void climberFunction(){
    
    if(rightStick.getRawButton(1)){

      climber.set(-0.7);

    }else if(leftStick.getRawButton(1)){

      climber.set(1);

    }else{

      climber.set(0);

    }
  }

  public void climber2Function(){

    if(rightStick.getRawButton(4)){
      
      climber2.set(0.8);

    }else if(rightStick.getRawButton(3)){

      climber2.set(-0.5);

    }else{

      climber2.set(0);

    }
  }
}
