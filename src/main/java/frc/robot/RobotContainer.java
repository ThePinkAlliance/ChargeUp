package frc.robot;

import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Navigate;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.Zero;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final ShuffleboardTab debugTab = Shuffleboard.getTab("debug");
        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

        PIDController xController = new PIDController(AutoConstants.kPXController, 0.5, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        Path file;

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                configureButtonBindings();

                debugTab.add(xController);
                debugTab.add(yController);
                debugTab.add(thetaController);

                file = Filesystem.getDeployDirectory().toPath().resolve("output/idk.wpilib.json");

                SmartDashboard.putNumber("distance", 1.524);
        }

        private void configureButtonBindings() {
                new JoystickButton(driverJoytick, 4).onTrue(
                                new Navigate(swerveSubsystem, new SwerveModulePosition(getDistance(), new Rotation2d()),
                                                1));
                new JoystickButton(driverJoytick, 3).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                new JoystickButton(driverJoytick, 2).onTrue(new Zero(swerveSubsystem));
        }

        private double getDistance() {
                return SmartDashboard.getNumber("distance", 1.524);
        }

        public Command getAutonomousCommand() {
                // 1. Create trajectory settings
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(DriveConstants.kDriveKinematics);

                // 2. Generate trajectory

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(),
                                new Pose2d(1.524, 0,
                                                Rotation2d.fromDegrees(180)),
                                trajectoryConfig);

                try {
                        trajectory = TrajectoryUtil.fromPathweaverJson(file);
                } catch (Exception er) {
                        er.printStackTrace();
                }

                swerveSubsystem.resetOdometry(trajectory.getInitialPose());

                // 3. Define PID controllers for tracking trajectory
                // Empty...

                // 4. Construct command to follow trajectory
                SwerveController swerveControllerCommand = new SwerveController(
                                trajectory,
                                () -> swerveSubsystem.getPose(),
                                DriveConstants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                                swerveSubsystem::setModuleStates,
                                swerveSubsystem);

                // 5. Add some init and wrap-up, and return everything
                return swerveControllerCommand.andThen(() -> swerveSubsystem.stopModules());
        }
}
