// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class USB{
        public static final int DRIVER_CONTROLLER = 0;      // Driver Controller USB ID
        public static final int OPERATOR_CONTROLLER = 1;    // Operator controller USB ID
        public static final int OPERATOR_LY = 1;
        public static final int OPERATOR_LX = 0;
        public static final int OPERATOR_RY = 5;
        public static final int OPERATOR_RX = 4;
        public static final int OPERATOR_RT = 3;
        public static final int OPERATOR_LT = 2;
    }

    public final class ModuleConstants{
        public static final double kWheelDiameterMeters = 0.10;
        public static final double kDriveMotorGearRatio = 1/ 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.42857142857143;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRPM2RadPerSec = kTurnEncoderRot2Rad / 60;
        public static final double kPTurning = 0.4;
    }

    public static final class DriveConstants {

        public static final double ArmEncoder2Meters = -20.5; // (51-35)/(3.536066-4.481552); // change in odometer x / change in arm encoder 

        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22);
        // Distance between front and back wheels

        public static final double kRobotRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 16;
        public static final int kFrontLeftTurningMotorPort = 15;

        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 11;

        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kFrontRightTurningMotorPort = 21;

        public static final int kBackRightDriveMotorPort = 26;
        public static final int kBackRightTurningMotorPort = 25;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = true;
        public static final boolean kBackRightDriveReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 17;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 22;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 27;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRot = -0.421387;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRot = -0.341309;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRot = 0.049805;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRot = -0.125977;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowButtonDriveModifier = 0.4;
        public static final double kSlowButtonTurnModifier = 0.5;
    }

    public static final class IntakeConstants {
        public static final int frontWheelID = 42;
        public static final int leftWheelID = 41;
        public static final int rightWheelID = 40;
    }

    public static final class ArmConstants {
        public static final int leftMotorID = 43;
        public static final int rightMotorID = 44;
        public static final double kP =  0.1;
        public static final double kD =  0.01;
        public static final double speakerEncoder = -35; //-30; TODO: CHANGE TO 30 FOR COMP
        public static final double farSpeakerEncoder = -50; // TODO: CHANGE FOR COMP
        public static final double ampEncoder = -140;
    }


    public static final class ClimbConstants {
        public static final int leftMotorID = 50;
        public static final int rightMotorID = 51;
        public static final double kP =  1 / 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kDriverRB = 6;

        public static final double kDeadband = 0.05;

        //BUTTONS

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int L3 = 9;
        public static final int R3 = 10;

        //AXES

        public static final int LX = 0;
        public static final int LY = 1;
        public static final int LT = 2;

        public static final int RT = 3;
        public static final int RX = 4;
        public static final int RY = 5;
    }

    public final class AutoShootState {
        public static final int RESET_ARM = 0;
        public static final int AIM = 1;
        public static final int SHOOT = 2;
        public static final int LOWER_AND_INTAKE = 3;
    }
}
