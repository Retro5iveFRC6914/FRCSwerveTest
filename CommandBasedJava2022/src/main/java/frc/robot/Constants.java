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

    public static final class speed {

        double shooterTicks = 5700;
        double indexSpeed = 0.7;
        double intakeSpeed = 0.9;

    }

    public static final class bools{

        boolean toggle = false;
        boolean shooterBool;

    }


    public static final class driveTrainIDs{

        //drivetrain IDs
        public static int leftFID = 1;
        public static int leftBID = 3;
        public static int rightFID = 2;
        public static int rightBID = 4;

    }


    public static final class mechanismIDs{

        //mechanism IDs
        public static int shooterID = 7;
        public static int climberID = 9;
        public static int climber2ID = 6;
        public static int indexID = 8;
        public static int intakeID = 5;

    }

    public static final class joystickIDs{

        //joystick IDs
        public static int leftStickID = 0;
        public static int rightStickID = 1;
        public static int mechPadID = 2;
        
    }


    public static final class PIDConstants{

        //PID numbers
        public static int kSlotIdx = 0;
        public static int kPIDLoopIdx = 0;
        public static int kTimeoutMs = 10;

    }



}
