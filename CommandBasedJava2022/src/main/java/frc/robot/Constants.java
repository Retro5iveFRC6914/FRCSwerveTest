// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    double shooterTicks = 5700;
    double indexSpeed = 0.7;
    double intakeSpeed = 0.9;

    boolean toggle = false;
    boolean shooterBool;

    //drivetrain IDs
    int leftFID = 1;
    int leftBID = 3;
    int rightFID = 2;
    int rightBID = 4;

    //mechanism IDs
    int shooterID = 7;
    int climberID = 9;
    int climber2ID = 6;
    int indexID = 8;
    int intakeID = 5;

    //joystick IDs
    int leftStickID = 0;
    int rightStickID = 1;
    int mechPadID = 2;

    //PID numbers
    int kSlotIdx = 0;
    int kPIDLoopIdx = 0;
    int kTimeoutMs = 10;


}
