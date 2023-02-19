package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.09);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.50;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(23.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 15;
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kBackRightDriveMotorPort = 17;

        public static final int kFrontLeftTurningMotorPort = 12;
        public static final int kBackLeftTurningMotorPort = 16;
        public static final int kFrontRightTurningMotorPort = 14;
        public static final int kBackRightTurningMotorPort = 18;

        public static final boolean kFrontLeftTurningReversed = false;
        public static final boolean kBackLeftTurningReversed = false;
        public static final boolean kFrontRightTurningReversed = false;
        public static final boolean kBackRightTurningReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 6;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 4;
        public static final int kBackRightDriveAbsoluteEncoderPort = 8;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        /**
         * These values where determined by lining up all the wheels and recording the
         * outputed positions.
         */
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =  2.91;//-2.68;//-1.48;//-0.99;// -3.09;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.98;//-1.80;// -1.08;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.35; //2.31;// 0.154;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.41;//-0.69;// 0.018;

        // This is the max speed without load.
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.0;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.75;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 3;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond
                * 0.35;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 1.5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kTowerControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.08;
    }
}
