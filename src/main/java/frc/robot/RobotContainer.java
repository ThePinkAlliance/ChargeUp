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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommandGroup;
import frc.robot.commands.Navigate;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.Zero;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraType;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final ShuffleboardTab debugTab = Shuffleboard.getTab("debug");
        private Trajectory trajectory = null;
        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
        private final CameraSubsystem cameraSubsystem = new CameraSubsystem(CameraType.LIMELIGHT);

        PIDController xController = new PIDController(AutoConstants.kPXController, 0.5, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        Path file;




  private final Command autoMoveCommand = new AutoCommandGroup(swerveSubsystem, cameraSubsystem);                                                          
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

                try {
                        // trajectory = TrajectoryUtil.fromPathweaverJson(file);
                } catch (Exception err) {
                        err.printStackTrace();
                }

                SmartDashboard.putNumber("distance", 2);
        }

        private void configureButtonBindings() {
                new JoystickButton(driverJoytick, 4).onTrue(
                                new Navigate(swerveSubsystem, new SwerveModulePosition(getDistance(), new Rotation2d()),
                                                1));
                new JoystickButton(driverJoytick, 3).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                new JoystickButton(driverJoytick, 2).onTrue(new Zero(swerveSubsystem));
        }

        private double getDistance() {
                return SmartDashboard.getNumber("distance", 2);
        }

        public Command getAutonomousCommand() {
                // 1. Create trajectory settings
                TrajectoryConfig trajectoryConfig2 = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond * 0.5,
                                1)
                                .setKinematics(DriveConstants.kDriveKinematics);

                // 2. Generate trajectory

                if (trajectory == null) {
                        trajectory = TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        List.of(
                                                        new Translation2d(2.5, 0),
                                                        new Translation2d(3, 0)),
                                        new Pose2d(4.7, 0,
                                                        Rotation2d.fromDegrees(0)),
                                        trajectoryConfig2);
                }

                // Reset the swerve subsystem pose to the inital pose of the trajectory.
                swerveSubsystem.resetOdometry(trajectory.getInitialPose());

                // 4. Construct command to follow trajectory
                SwerveController swerveControllerCommand = new SwerveController(
                                trajectory,
                                swerveSubsystem::getPose,
                                DriveConstants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                                swerveSubsystem::setModuleStates,
                                swerveSubsystem);

                // 5. Add some init and wrap-up, and return everything
                return new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("InitalPose",
                                                swerveSubsystem.getPose()
                                                                .toString()),
                                                swerveSubsystem),
                                swerveControllerCommand,
                                new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));
        }
}
