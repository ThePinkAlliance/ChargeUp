package frc.robot.commands.drive;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

        private final SwerveSubsystem swerveSubsystem;
        private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
        private final Supplier<Boolean> slowdown;
        private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

        public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
                        Supplier<Double> turningSpdFunction,
                        Supplier<Boolean> slowdown) {
                this.swerveSubsystem = swerveSubsystem;
                this.xSpdFunction = xSpdFunction;
                this.ySpdFunction = ySpdFunction;
                this.turningSpdFunction = turningSpdFunction;
                this.slowdown = slowdown;
                this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(
                                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(swerveSubsystem);
        }

        @Override
        public void initialize() {

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
                                ? xSpeed * (slowdown.get() ? Constants.DriveConstants.kTeleDriveSpeedReduction : 1)
                                : 0.0;
                ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed
                                * (slowdown.get() ? Constants.DriveConstants.kTeleDriveSpeedReduction : 1) : 0.0;
                turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

                // 3. Make the driving smoother
                xSpeed = (xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
                                * Constants.DriveConstants.kTeleDriveSpeedReduction;
                ySpeed = (yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
                                * Constants.DriveConstants.kTeleDriveSpeedReduction;
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

                if (slowdown.get()) {
                        swerveSubsystem.configureDriveRamp(.2);
                } else {
                        swerveSubsystem.configureDriveRamp(0);
                }

                // 6. Output each module states to wheels
                swerveSubsystem.setModuleStates(moduleStates);
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
