package frc.robot.commands.drive;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> slowDownSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private TrapezoidProfile.State xPreviousRef, yPreviousRef;
    private double profilePreviousTime;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> slowdownSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slowDownSupplier = slowdownSupplier;
        this.profilePreviousTime = 0;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        xPreviousRef = new TrapezoidProfile.State(0, 0);
        yPreviousRef = new TrapezoidProfile.State(0, 0);
        profilePreviousTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = Math.copySign(xSpdFunction.get(), xSpdFunction.get()); // this does not square
        double ySpeed = Math.copySign(ySpdFunction.get(), ySpdFunction.get()); // this does not square
        xSpeed = xSpeed * Math.abs(xSpeed);
        ySpeed = ySpeed * Math.abs(ySpeed);

        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband
                ? (xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
                        * (slowDownSupplier.get() ? Constants.DriveConstants.kTeleDriveSpeedReduction : 1)
                : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband
                ? (ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) * (slowDownSupplier.get()
                        ? Constants.DriveConstants.kTeleDriveSpeedReduction
                        : 1)
                : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        TrapezoidProfile xProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                        Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond),
                new TrapezoidProfile.State(xSpeed, 0), xPreviousRef);
        TrapezoidProfile yProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                        Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond),
                new TrapezoidProfile.State(ySpeed, 0), yPreviousRef);

        double currentTime = Timer.getFPGATimestamp() - profilePreviousTime;
        TrapezoidProfile.State xState = xProfile.calculate(currentTime);
        TrapezoidProfile.State yState = yProfile.calculate(currentTime);

        xSpeed = xState.position;
        ySpeed = yState.position;

        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        ySpeed = ySpeed * -1.0;

        /**
         * Drive relative to the field.
         */
        Rotation2d robotAngle = swerveSubsystem.getRotation2d();
        double x = xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin();
        double y = xSpeed * robotAngle.getSin() + ySpeed * -robotAngle.getCos();

        SmartDashboard.putNumber("x_speed", x);
        SmartDashboard.putNumber("y_speed", y);

        chassisSpeeds = new ChassisSpeeds(x, y, turningSpeed);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        xPreviousRef = xState;
        yPreviousRef = yState;
        profilePreviousTime = Timer.getFPGATimestamp();
    }

    public void configureKp(double kP) {
        swerveSubsystem.setKp(kP);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
