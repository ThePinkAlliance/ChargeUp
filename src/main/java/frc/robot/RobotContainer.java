package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.JoystickArm;
import frc.robot.commands.arm.pivot.PivotToDegree;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.commands.arm.turret.RotateToDegreeProfiled;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.manipulator.JoystickManipulator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final ShuffleboardTab debugTab = Shuffleboard.getTab("debug");
        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
        private final Joystick towerJoytick = new Joystick(OIConstants.kTowerControllerPort);

        private SendableChooser<Trajectory> autoSendable = new SendableChooser<Trajectory>();

        public static PIDController xController = new PIDController(AutoConstants.kPXController, 0.5, 0);
        public static PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        public static ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

        // Tower
        private final ArmSubsystem armSubsystem = new ArmSubsystem(41, 42, 9,
                        72.16, 1,
                        0.5, new Constraints(0, 0));
        private final TurretSubsystem turretSubsystem = new TurretSubsystem(31);
        private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(43, 44);

        public RobotContainer() {
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                configureControllerBindings();

                SmartDashboard.putData("Auto Chooser", autoSendable);

                debugTab.add(xController);
                debugTab.add(yController);
                debugTab.add(thetaController);

                try {
                        Trajectory idk = TrajectoryUtil.fromPathweaverJson(
                                        Filesystem.getDeployDirectory().toPath().resolve("output/idk.wpilib.json"));
                        Trajectory e1 = TrajectoryUtil.fromPathweaverJson(
                                        Filesystem.getDeployDirectory().toPath().resolve("output/E1.wpilib.json"));
                        Trajectory e2 = TrajectoryUtil.fromPathweaverJson(
                                        Filesystem.getDeployDirectory().toPath().resolve("output/E2.wpilib.json"));

                        autoSendable.addOption("e1", e1);
                        autoSendable.addOption("e2", e2);
                        autoSendable.setDefaultOption("idk", idk);
                } catch (Exception err) {
                        err.printStackTrace();
                }
        }

        private void configureControllerBindings() {
                // Base
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

                // manipulatorSubsystem.setDefaultCommand(
                // new JoystickManipulator(manipulatorSubsystem, () ->
                // towerJoytick.getRawAxis(0),
                // () -> towerJoytick.getRawAxis(4)));

                armSubsystem.setDefaultCommand(
                                new JoystickArm(armSubsystem, () -> towerJoytick.getRawAxis(1),
                                                () -> towerJoytick.getRawAxis(5) / 2));

                new JoystickButton(driverJoytick, 5).onTrue(new PivotToDegree(armSubsystem,
                                90));
                new JoystickButton(driverJoytick, 6).onTrue(new PivotToDegree(armSubsystem,
                                180));
                // new JoystickButton(driverJoytick, 4)
                // .onTrue(new RotateToDegree(turretSubsystem, 270, () ->
                // armSubsystem.getPivotAngle()));
                // new JoystickButton(driverJoytick, 3)
                // .onTrue(new RotateToDegree(turretSubsystem, 180, () ->
                // armSubsystem.getPivotAngle()));
                // new JoystickButton(driverJoytick, 2)
                // .onTrue(new RotateToDegree(turretSubsystem, 0, () ->
                // armSubsystem.getPivotAngle()));
                new JoystickButton(driverJoytick, 1).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        }

        public Command getAutonomousCommand() {
                Trajectory trajectory = autoSendable.getSelected();

                // Reset the swerve subsystem pose to the inital pose of the trajectory.
                swerveSubsystem.resetOdometry(trajectory.getInitialPose());

                // 4. Construct command to follow trajectory
                Command swerveControllerCommand = swerveSubsystem.buildSwerveCommand(trajectory);

                // 5. Add some init and wrap-up, and return everything
                return new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("InitalPose",
                                                swerveSubsystem.getPose()
                                                                .toString()),
                                                swerveSubsystem),
                                swerveControllerCommand.alongWith(),
                                new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));
        }
}
