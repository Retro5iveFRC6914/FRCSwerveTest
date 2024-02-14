// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
public class Intake extends SubsystemBase {


  // Intake arms that open and close
  CANSparkMax intakeInhale;
  CANSparkMax intakeExhale;
  AbsoluteEncoder revboreEncoder ;
  private static Intake _instance;

  private DigitalInput sensor;

  // Toggling
  private boolean toggledUp;
  private SparkMaxPIDController hingePIDController;

  // Spaghetti wheels
  WPI_TalonSRX intake;

  /** Creates a new Intake. */
  public Intake() {
    toggledUp = true;
    // Hinges
    intakeInhale = new CANSparkMax(Constants.IntakeConstants.intakeInhaleId, MotorType.kBrushless);
    intakeExhale = new CANSparkMax(Constants.IntakeConstants.intakeExhaleId, MotorType.kBrushless);


    intakeInhale.restoreFactoryDefaults();
    intakeExhale.restoreFactoryDefaults();
    intakeInhale.setInverted(true);
    
    intakeExhale.follow(intakeInhale, true);

    intakeExhale.setIdleMode(IdleMode.kBrake);
    intakeInhale.setIdleMode(IdleMode.kBrake);

    
    

    revboreEncoder = intakeInhale.getAbsoluteEncoder(Type.kDutyCycle);
    revboreEncoder.setInverted(true);

  // PIDS!
  hingePIDController = intakeInhale.getPIDController();
  hingePIDController.setFeedbackDevice(revboreEncoder);
  hingePIDController.setP(Constants.IntakeConstants.hingeP);
  hingePIDController.setI(Constants.IntakeConstants.hingeI);
  hingePIDController.setD(Constants.IntakeConstants.hingeD);
  hingePIDController.setOutputRange(-1.0, 1.0);
  intakeInhale.setClosedLoopRampRate(0.5);
  intakeExhale.setClosedLoopRampRate(0.5);


  // Spaghetti
  intake = new WPI_TalonSRX(Constants.IntakeConstants.intakeId);
  intake.enableVoltageCompensation(true);

  sensor = new DigitalInput(Constants.IntakeConstants.sensorId);
}



public static Intake getInstance() {
  if (_instance == null) {
          _instance = new Intake();
  }
  return _instance;
}
/**
* @return Hinge encoder position. 
*/
public double getEncoder() {
  return revboreEncoder.getPosition();
}

public void zeroHinge() {
}
/**
* Sets the hinge power of the left and right motor.
* @param leftPower Power of the left hinge motor.
* @param rightPower Power of the right hinge motor.
*/
public void setHinge(double leftPower, double rightPower) {
  intakeInhale.set(leftPower);
}

public void runIntake() {
}
/** Sets speed of the intake. */
public void runIntake(double speed) {
  intake.set(speed);
}
/** Vomits the game pieces out. */
public void vomit() {
  intake.set(Constants.IntakeConstants.vomitSpeed);
}
/** Stops intake. */
public void stopIntake() {
  intake.set(0);
}
/** Sets hinge to a position with PID looping. */
public void hingeTo(double position) {
  hingePIDController.setReference(position, CANSparkMax.ControlType.kPosition);
}
/** gets the state for the intake sensor. */
public boolean getIntakeSensor() {
  return !sensor.get();
}



  

   /**
   * Puts rev bore encoder positions on smartdashboard.
   * Puts intake sensor state on smartdashboard.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("Encoder Position", getEncoder());
    SmartDashboard.putBoolean("intake sensor", getIntakeSensor());
  }
}
    // This method will be called once per scheduler run


    