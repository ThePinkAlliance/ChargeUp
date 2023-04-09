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
                public static final boolean kBackLeftDriveEncoderReversed = false;
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
                public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.90; // 1.9036
                public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.90; // -1.904
                public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.56; // 1.5615
                public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.49; // 1.49563

                // This is the max speed without load.
                public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
                public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1; // 0.96
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                                / 2.8;
                public static double kTeleDriveSpeedReduction = .4;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.5;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.5;
        }

        public static final class TurretConstants {
                public static final double TURRET_OFFSET = 160;
                public static final double ANGLE_TOLERANCE_FOR_EXTERNAL_SENSOR = 0.5;
                public static final GainsFX TURRET_GAINS_FX = new GainsFX(0.01, 0.0, 0, 0.0, 0, 0);
                public static final double FULL_MOTOR_ROTATIONS = 686;
        }

        public static final class GrabberConstants {
                public static final int GRABBER_CAN_ID_GRASP = 43;
                public static final int GRABBER_CAN_ID_INTAKE = 44;
                public static final int GRABBER_INTAKE_CURRENT_LIMIT = 25;
                public static final int BETTER_GRABBER_INTAKE_CURRENT_LIMIT = 6;
                public static final int GRABBER_GRASP_CURRENT_LIMIT = 40;
                public static final double GRABBER_GRASP_OPEN_WATCHDOG = 0.5;
                public static final double GRABBER_GRASP_OPEN_POWER = 0.5;
                public static final double GRABBER_GRASP_CLOSE_POWER = 0.7;
                public static final double GRABBER_GRASP_SUSTAINED_CURRENT_OPEN_LIMIT = 30;
                public static final double GRABBER_GRASP_SUSTAINED_CURRENT_OPEN_TIMEOUT = 0.2;
                public static final double GRABBER_GRASP_SUSTAINED_CURRENT_CLOSE_TIMEOUT = 0.3;
                public static final GainsFX GRASP_GAINS_FX = new GainsFX(0.2, 0.0, 0, 0.0, 0, 0);
                public static final GainsFX INTAKE_GAINS_FX = new GainsFX(0.01, 0.0, 0, 0.0, 0, 0);

                public static final double GRASP_OPEN_POSITION = 0;
                public static final double GRASP_CLOSED_POSITION = -20;

                public static final double GRABBER_CONE_CLOSED = 0.5;
                public static final double GRABBER_CUBE_CLOSED = 1;
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

                public static final double kDeadband = 0.05;

                public static final int kButtonStart = 8;
                public static final int kButtonBack = 7;
                public static final int kButtonY = 4;
                public static final int kButtonB = 2;
                public static final int kButtonX = 3;
                public static final int kButtonA = 1;
                public static final int kButtonLeftBumper = 5;
                public static final int kButtonRightBumper = 6;

                public static boolean cubeSelected = false;
        }

        public static final class ArmConstants {
                private static double VELOCITY_MULTIPLER = 1.2;
                private static double ACCEL_MULTIPLER = 2.5;

                // kP was .22
                public static final GainsFX MOTIONM_GAINS_FX = new GainsFX(0.205, 0, 0, 0.3, 0, 1);
                public static final GainsFX POSITION_GAINS_FX = new GainsFX(0.1, 0, 0, 0.3, 0, 1);
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
                public static final double POWER_LIMIT_PIVOT = 1;

                public static final double COLLECT_CONE_ANGLE_STAGE_ONE = 82.5;
                public static final double COLLECT_STOW = 180.0;

                public static final double LED_SPEED = 0.3;

                public static final int kSlotIdx = 0;
                public static final int kSlotId_ForPosition = 1;
                public static final int kPIDLoopIdx = 0;
                public static final int kTimeoutMs = 10;
                public static final int kPeriodMs = 10;
                public static final double kPercentDeadband = 0.001;
                public static final double kNominalForwardPeak = 0.7;
                public static final double kNominalReversePeak = -0.7;
                public static final double kAllowableCloseLoopError = 27.0;
        }

        public static final class ExtenderConstants {
                public static final float EXTENDER_90_MAX_LIMIT = 112f; // 54.0f
                public static final float EXTENDER_MIN_LIMIT = 3.0f;
                public static final double EXTENDER_MARGIN_OF_ERROR = 3.0;
                public static final double POWER_LIMIT_EXTEND = 1;
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
