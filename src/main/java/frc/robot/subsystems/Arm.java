// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;



public class Arm extends SubsystemBase {

  private boolean isReadyToRecieve = false;

 
  private CANSparkMax Rotate1;
  private CANSparkMax Rotate2;

  private static Arm _instance;
  private AbsoluteEncoder Rotate1Encoder;
  private AbsoluteEncoder Rotate2Encoder;

  // PIDS!
  private SparkMaxPIDController Rotate2_pidController;
  private SparkMaxPIDController Rotate1_pidController;

  private RelativeEncoder backupencoder;

  private static final int kCPR = 8192;
  
  /** Creates a new Arm. */
  public Arm() {
    
    Rotate1 = new CANSparkMax(Constants.ArmConstants.Rotate1ID, MotorType.kBrushless);
    Rotate2 = new CANSparkMax(Constants.ArmConstants.Rotate2Id, MotorType.kBrushless);
    
    Rotate1.restoreFactoryDefaults();
   
    Rotate2.restoreFactoryDefaults();

    
    Rotate1.setInverted(true);

    

    //Arm encoders
    Rotate1Encoder = Rotate1.getAbsoluteEncoder(Type.kDutyCycle);
    Rotate2Encoder.setInverted(false);

    Rotate1Encoder = Rotate2.getAbsoluteEncoder(Type.kDutyCycle);
    Rotate2Encoder.setInverted(true);


    backupencoder = Rotate2.getEncoder();
    backupencoder.setPosition(0);
    
    Rotate1Encoder.setZeroOffset(45/360);
    // PID Controllers

    // Rotate2 PID
    Rotate2_pidController = Rotate2.getPIDController();
    Rotate2_pidController.setFeedbackDevice(Rotate2Encoder);
    Rotate2_pidController.setP(Constants.ArmConstants.Rotate2UpP);
    Rotate2_pidController.setI(Constants.ArmConstants.Rotate2I);
    Rotate2_pidController.setD(Constants.ArmConstants.Rotate2D);
    Rotate2_pidController.setOutputRange(-1.0, 1.0);

    

    // Rotate1 PID
    Rotate1_pidController = Rotate1.getPIDController();
    Rotate1_pidController.setFeedbackDevice(Rotate1Encoder);
    Rotate1_pidController.setP(Constants.ArmConstants.Rotate1UpP);
    Rotate1_pidController.setI(Constants.ArmConstants.Rotate1I);
    Rotate1_pidController.setD(Constants.ArmConstants.Rotate1D);
    
    Rotate1_pidController.setOutputRange(-1.0, 1.0);


    // Brake mode
    
    Rotate1.setIdleMode(IdleMode.kBrake);
    Rotate1.setIdleMode(IdleMode.kBrake);
    Rotate1.setClosedLoopRampRate(0.25);

    //reset encoders
   


  }

  public static Arm getInstance() {
    if (_instance == null) {
      _instance = new Arm();
    }
    return _instance;
  }

  
/**
 * Sets Rotate1 to a power.
 * @param power power of elbow
 */
  public void setRotate1(double power) {
    Rotate1.set(power *.7);
  }

/**
 * Stops Rotate1.
 */
  public void stopRotate1() {
    Rotate1.set(0);
  }
/**
 * @return Rotate1 Encoder in Degrees
 */
  public double getRotate1Clicks() {
    return Rotate1Encoder.getPosition()*360;
  }

  /**
   * Gets Rotate2 Encoder Position.
   */
  public double getRotate2Clicks() {
    return Rotate2Encoder.getPosition();
  }

  // Rotate2

  /**
   * @param power Power of Rotate2
   */
  public void moveRotate2(double power) {
    Rotate2.set(power);
  }

  public void stopRotate2() {
    Rotate2.set(0);
  }

 /**
  * Uses PID looping to set the Rotate2 to a certain position 
  * @param refrence Position to send Rotate2 to 
  */
  public void clawTo(double refrence) {
    Rotate2_pidController.setReference(refrence, ControlType.kPosition);
    
  }

  /**
   * Uses PID looping to set the Rotate1 to a certain position 
   * @param refrence Position to send Rotate1 to 
   */
  public void Rotate1To(double refrence) {
    Rotate1_pidController.setReference(refrence/360, CANSparkMax.ControlType.kPosition);
  }


/**
 * Sets the PID values for Rotate1 going up
 */
  public void setRotate1PID() {
    Rotate1_pidController.setP(Constants.ArmConstants.Rotate1UpP);
    Rotate1_pidController.setI(Constants.ArmConstants.Rotate1I);
    Rotate1_pidController.setD(Constants.ArmConstants.Rotate1D);
  }
  /**
   * Sets the PID value for the Rotate1 when it goes down
   */
  public void setRotate2PID() {
    Rotate2_pidController.setP(Constants.ArmConstants.Rotate2DownP);
    Rotate2_pidController.setI(Constants.ArmConstants.Rotate2I);
    Rotate2_pidController.setD(Constants.ArmConstants.Rotate2D);
  }

  public void setReadyToRecieve() {
    isReadyToRecieve = true;
  }

  public void notReadyToRecieve() {
    isReadyToRecieve = false;
  }
/**
 * @return if the robot is in the ready to recieve position
 */
  public boolean getReadyToRecieve() {
    return isReadyToRecieve;
  }
/**
*puts the encoder positions for the Rotate1 and the Rotate2 on the smart dashboard
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotate1Encoder", getRotate1Clicks());
    SmartDashboard.putNumber("Rotate2Encoder", getRotate2Clicks());
    SmartDashboard.putNumber("Rotate2 current", Rotate2.getOutputCurrent());
    
    
  }
}







 /* 
   * Declaring the Pigeon 2 gyro. Creating the variables to recieve yaw and pitch
   * values from the sensor
   * 
   * Declaring the CANcoder for the Falcon 500 running the arm.
   */
  /*double getArmPos = 0;

  CANCoder armSensor = new CANCoder(1);
 
  public double getPosition() {
    return getArmPos;
  }
 public void MotorLimit(){
  if (arm.get() > 170){

  arm.stopMotor();

   } else if(arm.get() > ){
arm.stopMotor();
  }
}
*/