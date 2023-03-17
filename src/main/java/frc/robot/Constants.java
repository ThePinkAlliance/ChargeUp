package frc.robot;

import com.ThePinkAlliance.core.util.GainsFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static final class ModuleConstants {
                public static final double kWheelDiameterMeters = Units.inchesToMeters(4.09);
                public static final double kDriveMotorGearRatio = 1 / 8.14;
                public static final double kTurningMotorGearRatio = 1 / 12.8;
                public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
                                * kWheelDiameterMeters;
                public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
                public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
                public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
                public static final double kPTurning = 0.47;
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

                public static final boolean kFrontLeftDriveEncoderReversed = true;
                public static final boolean kBackLeftDriveEncoderReversed = true;
                public static final boolean kFrontRightDriveEncoderReversed = true;
                public static final boolean kBackRightDriveEncoderReversed = true;

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
                public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.93;
                public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.20;
                public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.57;
                public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.47;

                // This is the max speed without load.
                public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
                public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.87;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                                / 2.8;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.2;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.5;
        }

        public static final class TurretConstants {
                public static final double TURRET_OFFSET = 160;
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

                public static final int kYAxis = 1;
                public static final int kXAxis = 0;
                public static final int kDriverRotAxis = 4;
                public static final int kTowerPivotAxis = 5;
                public static final int kTowerExtendAxis = 1;
                public static final int kTowerManipulatorRightAxis = 4;
                public static final int kTowerManipulatorLeftAxis = 0;
                public static final int lTowerTurretAxis = 4;

                public static final int kDriverFieldOrientedButtonIdx = 1;

                public static final double kDeadband = 0.08;

                public static final int kButtonStart = 8;
                public static final int kButtonBack = 7;
                public static final int kButtonY = 4;
                public static final int kButtonB = 2;
                public static final int kButtonX = 3;
                public static final int kButtonA = 1;
                public static final int kButtonLeftBumper = 5;
                public static final int kButtonRightBumper = 6;
        }

        public static final class ArmConstants {
                private static double VELOCITY_MULTIPLER = 1.2;
                private static double ACCEL_MULTIPLER = 2.5;

                public static final GainsFX MOTIONM_GAINS_FX = new GainsFX(0.20, 0, 0, 0.3, 0, 1);
                public static final double MAX_CRUISE_VELOCITY = 36864 * VELOCITY_MULTIPLER;
                /**
                 * Old Value 18480; The acceleration needs to be tuned so it can accel and
                 * decel properly.
                 * 
                 * The acceleration is mechanism dependent so it needs to be tuned. keep in mind
                 * that the acceleration should be big enough for mechanism to accelerate and
                 * decelerate without surpassing it's target setpoint.
                 */
                public static final double MAX_ACCELERATION = 6064 * ACCEL_MULTIPLER;
                public static final double PITCH_FLOOR_ABSOLUTE = 131.66;
                public static final double PITCH_FLOOR_OFFSET = 71.11;
                public static final double POWER_LIMIT_EXTEND = 1;
                public static final double POWER_LIMIT_PIVOT = 1;
                public static final float EXTENDER_90_MAX_LIMIT = 54.0f;
                public static final float EXTENDER_MIN_LIMIT = 3.0f;

                public static final double COLLECT_CONE_ANGLE_STAGE_ONE = 82.5;
                public static final double COLLECT_STOW = 180.0;

        }

        public static final class ManipulatorConstants {
                public static final double CONE_LEFT = 24.19;
                public static final double CONE_RIGHT = 23.19;

                public static final double CUBE_LEFT = 14.19;
                public static final double CUBE_RIGHT = 14.19;

                public static final double COLLECT_CONE_FULLY_OPEN = 0;

                public static final double CONE_GRIP_MULTIPLER = 8;
                public static final double CUBE_GRIP_MULTIPLER = 4;

                // KnockConeLeft
                public static final double COLLECT_CONE_LEFT_LEFT_STAGE_ONE = 37.97;
                public static final double COLLECT_CONE_LEFT_RIGHT_STAGE_TWO = 56.0;
                // KnockConeRight
                public static final double COLLECT_CONE_RIGHT_RIGHT_STAGE_ONE = 38.16;
                public static final double COLLECT_CONE_RIGHT_LEFT_STAGE_TWO = 58.0;

        }
}
