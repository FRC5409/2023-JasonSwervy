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

    public static final class kControllers {
        public static final int kPrimaryController                  = 0;
        public static final int kSecondaryController                = 1;
    }

    public static final class kCANID {
        // All placeholder values

        public static final int kDriveMotor1                        = 1;
        public static final int kDriveMotor2                        = 2;
        public static final int kDriveMotor3                        = 3;
        public static final int kDriveMotor4                        = 4;

        public static final int kTurnMotor1                         = 5;
        public static final int kTurnMotor2                         = 6;
        public static final int kTurnMotor3                         = 7;
        public static final int kTurnMotor4                         = 8;

        public static final int kDriveEncoder1                      = 9;
        public static final int kDriveEncoder2                      = 10;
        public static final int kDriveEncoder3                      = 11;
        public static final int kDriveEncoder4                      = 12;

        public static final int kTurnEncoder1                       = 13;
        public static final int kTurnEncoder2                       = 14;
        public static final int kTurnEncoder3                       = 15;
        public static final int kTurnEncoder4                       = 16;

        public static final int kGyro                               = 17;

    }

    public static final class kRobot {
        public static final double length                           = 0.6;
        public static final double width                            = 0.6;
    }

    public static final class kDrive {

        public static final double kWheelRadius                     = 0.0475;
        public static final double kWheelDiameter                   = 2 * kWheelRadius;
        public static final double kWheelCircumference              = Math.PI * kWheelDiameter;

        public static final double kMaxDriveVelocity                = 4;
        public static final double kMaxDriveAngularVelocity         = Math.PI; // half rotation per second
        public static final double kMaxTurnAngularAcceleration      = 2 * Math.PI; // radius per second squared

        public static final class kEncoder {
            public static final double kEncoderCPR                  = 4096;

            public static final double kDriveSensorCoefficient      = kWheelCircumference / kEncoderCPR;
            public static final double kTurnSensorCoefficient       = 2 * Math.PI / kEncoderCPR;
        public static final String kUnitString                      = "m";
        }

        // TODO: Must be tuned
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/index.html
        public static final double kDriveP                          = 0;
        public static final double kDriveI                          = 0;
        public static final double kDriveD                          = 0;
        public static final double kDriveFFkS                       = 0;
        public static final double kDriveFFkV                       = 0;

        public static final double kTurnP                           = 0;
        public static final double kTurnI                           = 0;
        public static final double kTurnD                           = 0;
        public static final double kTurnFFkS                        = 0;
        public static final double kTurnFFkV                        = 0;

    }

}
