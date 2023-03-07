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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.JoystickArm;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.pivot.PivotToDegreeMagic;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.manipulator.JoystickManipulator;
import frc.robot.commands.manipulator.GoToPositionManipulator;
import frc.robot.commands.manipulator.ZeroManipulator;
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
                        1, new Constraints(0, 0));
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
                        /**
                         * These are all testing trajectories for pathweaver.
                         */
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
                /* Drivetrain (Base) */
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

                /* Arm Controls (Base) */
                armSubsystem.setDefaultCommand(
                                new JoystickArm(armSubsystem, () -> towerJoytick.getRawAxis(1),
                                                () -> towerJoytick.getRawAxis(5) / 2));

                manipulatorSubsystem.setDefaultCommand(new JoystickManipulator(manipulatorSubsystem,
                                () -> towerJoytick.getRawAxis(4), () -> towerJoytick.getRawAxis(0)));

                new JoystickButton(driverJoytick, 5).onTrue(
                                new PivotToDegreeMagic(180,
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 2,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> !turretSubsystem.isMoving(),
                                                armSubsystem));
                new JoystickButton(driverJoytick, 6).onTrue(
                                new PivotToDegreeMagic(85,
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 2,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> !turretSubsystem.isMoving(),
                                                armSubsystem));

                /* Extend Controls (Base) */
                new JoystickButton(towerJoytick, 4)
                                .onTrue(new ExtendTicks(70, armSubsystem));

                new JoystickButton(driverJoytick, 4).onTrue(new GoToPositionManipulator(16, manipulatorSubsystem));
                new JoystickButton(driverJoytick, 3).onTrue(new GoToPositionManipulator(2, manipulatorSubsystem));
                new JoystickButton(driverJoytick, 2).onTrue(new ZeroManipulator(manipulatorSubsystem));

                new JoystickButton(towerJoytick, 3)
                                .onTrue(new ExtendTicks(35, armSubsystem));

                new JoystickButton(towerJoytick, 2)
                                .onTrue(new ExtendTicks(1, armSubsystem));

                new JoystickButton(driverJoytick, 1)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                /* Turret Controls (Base) */
                new POVButton(driverJoytick, 90).onTrue(new RotateToDegree(turretSubsystem, 90,
                                () -> armSubsystem.getArmPitch() > 90));

                new POVButton(driverJoytick, 180)
                                .onTrue(new RotateToDegree(turretSubsystem, 180,
                                                () -> armSubsystem.getArmPitch() > 90));

                new POVButton(driverJoytick, 270)
                                .onTrue(new RotateToDegree(turretSubsystem,
                                                270,
                                                () -> armSubsystem.getArmPitch() > 90));

                new POVButton(driverJoytick, 0)
                                .onTrue(new RotateToDegree(turretSubsystem, 0, () -> armSubsystem.getArmPitch() > 100));
        }

        public void onDisabledInit() {
                armSubsystem.setPositionToHold(0);
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
