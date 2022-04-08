// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace oiConstants {

    //drivetrain
    constexpr int fLeftID = 0;
    constexpr int bLeftID = 0;
    constexpr int fRightID = 0;
    constexpr int bRightID = 0;

    //shooter
    constexpr int shooterID = 0;
    
    //index and intake
    constexpr int intakeID = 0;
    constexpr int indexID = 0;

    //climbers
    constexpr int primaryClimberID = 0;
    constexpr int secondaryClimberID = 0;

    //controls
    constexpr int rightS = 0;
    constexpr int leftS = 0;
    constexpr int mPad = 0;
}