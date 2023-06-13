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

        public static final int kDriveMotor1                        = 12;
        public static final int kDriveMotor2                        = 9;
        public static final int kDriveMotor3                        = 15;
        public static final int kDriveMotor4                        = 18;

        public static final int kTurnMotor1                         = 11;
        public static final int kTurnMotor2                         = 8;
        public static final int kTurnMotor3                         = 14;
        public static final int kTurnMotor4                         = 17;

        public static final int kCANCoder1                          = 13;
        public static final int kCANCoder2                          = 10;
        public static final int kCANCoder3                          = 16;
        public static final int kCANCoder4                          = 19;

        public static final int kGyro                               = 3;

    }

    public static final class kRobot {
        public static final double length                           = 0.64;
        public static final double width                            = 0.64;
    }

    public static final class kDrive {

        public static final double kWheelRadius                     = 0.05; // metres
        public static final double kWheelDiameter                   = 2 * kWheelRadius;
        public static final double kWheelCircumference              = Math.PI * kWheelDiameter;

        public static final double kDriveGearRatio                  = 6.75;
        public static final double kTurnGearRatio                   = 150.0 / 7.0;

        // public static final double kMaxDriveVelocity                = 4; // metres per second
        // public static final double kMaxTurnAngularVelocity          = 2 * Math.toRadians(360); // rotation per second
        // public static final double kMaxTurnAngularAcceleration      = 2 * Math.toRadians(360); // rotation per second squared
        public static final double kMaxDriveVelocity                = 4.56; // metres per second
        public static final double kMaxTurnAngularVelocity          = 10; // rotation per second
        public static final double kMaxTurnAngularAcceleration      = 2 * Math.toRadians(360); // rotation per second squared

        public static final int kDriveMotorCurrentLimit             = 40;
        public static final int kTurnMotorCurrentLimit              = 30;

        public static final double kXSpeedDeadband                  = 0.125;
        public static final double kYSpeedDeadband                  = 0.125;
        public static final double kTargetHeadingDeadband           = 0.25;
        public static final double kManualRotationDeadband          = 0.2;

        public static final double kXSpeedSlewRate                  = 2.2;
        public static final double kYSpeedSlewRate                  = 2.2;
        public static final double kManualRotationSlewRate          = 2.2;

        public static final double kHeadingSnap                     = Math.toRadians(45);

        public static final class kRelativeEncoder {
            public static final double kCPR                         = 42;

            public static final double kDriveSensorCoefficient      = kWheelCircumference / kDriveGearRatio;
            public static final double kTurnSensorCoefficient       = 2 * Math.PI / kTurnGearRatio;
        }

        public static final class kCANCoder {
            public static final double kCPR                         = 4096;

            public static final double kAbsoluteEncoderOffset1      = 327.744;
            public static final double kAbsoluteEncoderOffset2      = 20.830;
            public static final double kAbsoluteEncoderOffset3      = 205.137;
            public static final double kAbsoluteEncoderOffset4      = 76.641;
        }

        public static enum Location {
            TopLeft,
            TopRight,
            BottomLeft,
            BottomRight
        }

        public static final double kDriveP                          = 0.04;
        public static final double kDriveI                          = 0;
        public static final double kDriveD                          = 0;
        public static final double kDriveFF                         = 0.22;

        public static final double kTurnP                           = 0.07;
        public static final double kTurnI                           = 0;
        public static final double kTurnD                           = 0;
        public static final double kTurnFF                          = 0;

        public static final double kHeadingP                        = 3.3;
        public static final double kHeadingI                        = 0;
        public static final double kHeadingD                        = 0;

    }

}
