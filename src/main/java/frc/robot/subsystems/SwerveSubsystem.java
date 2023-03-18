package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveController;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed, "base");

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed, "base");

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed, "base");

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed, "base");

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Field2d field2d = new Field2d();
    private final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            getRotation2d(), new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            }, new Pose2d());
    private Pose2d currentPose2d = new Pose2d();

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        SmartDashboard.putData(field2d);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public List<SwerveModulePosition> getPositions() {
        return List.of(frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition());
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * NOTE: This method has been returning incorrect positions when using
     * SwerveControllerCommand
     */
    public Pose2d getPose() {
        return currentPose2d;
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    /*
     * Set the heading of all the swerve pods.
     */
    public void setAllHeadings(double heading) {
        frontLeft.setDesiredState(
                new SwerveModuleState(frontLeft.getState().speedMetersPerSecond, new Rotation2d(heading)));
        frontRight.setDesiredState(
                new SwerveModuleState(frontRight.getState().speedMetersPerSecond, new Rotation2d(heading)));
        backLeft.setDesiredState(
                new SwerveModuleState(backLeft.getState().speedMetersPerSecond, new Rotation2d(heading)));
        backRight.setDesiredState(
                new SwerveModuleState(backRight.getState().speedMetersPerSecond, new Rotation2d(heading)));
    }

    public double getPitch() {
        return this.gyro.getPitch();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        this.estimator.addVisionMeasurement(pose, timestamp);
    }

    public void setKp(double kP) {
        frontLeft.setKp(kP);
        frontRight.setKp(kP);
        backLeft.setKp(kP);
        backRight.setKp(kP);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Left Angle", frontLeft.getPosition().angle.getRadians());

        SmartDashboard.putNumber("Base Pitch", getPitch());

        currentPose2d = estimator.update(getRotation2d(),
                new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                        backRight.getPosition() });

        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putNumber("Robot Location X",
        // getPose().getTranslation().getX());
        // SmartDashboard.putNumber("Robot Location Y",
        // getPose().getTranslation().getY());
        // SmartDashboard.putNumber("Front Left Rotation (canID=2):",
        // frontLeft.getRawAbsoluteAngularPosition());
        // SmartDashboard.putNumber("Front Right Rotation (canID=4):",
        // frontRight.getRawAbsoluteAngularPosition());
        // SmartDashboard.putNumber("Back Left Rotation (canID=6):",
        // backLeft.getRawAbsoluteAngularPosition());
        // SmartDashboard.putNumber("Back Right Rotation (canID=8):",
        // backRight.getRawAbsoluteAngularPosition());

        field2d.setRobotPose(currentPose2d);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public Command buildSwerveCommand(Trajectory trajectory) {
        return new SwerveController(
                trajectory,
                this::getPose,
                DriveConstants.kDriveKinematics,
                RobotContainer.xController,
                RobotContainer.yController,
                RobotContainer.thetaController,
                this::setModuleStates,
                this);
    }

    public void move(double vx, double vy, double radsPerSecond) {
        setModuleStates(Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(new ChassisSpeeds(vx, vy, radsPerSecond)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
