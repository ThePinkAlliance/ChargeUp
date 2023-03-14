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
import frc.robot.commands.AprilTagMoverCommand;
import frc.robot.commands.arm.JoystickArm;
import frc.robot.commands.arm.JoystickArmMagic;
import frc.robot.commands.arm.KnockConeLeftStageOne;
import frc.robot.commands.arm.KnockConeLeftStageTwo;
import frc.robot.commands.arm.KnockConeRightStageOne;
import frc.robot.commands.arm.KnockConeRightStageTwo;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.pivot.PivotToDegreeMagic;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.manipulator.JoystickManipulator;
import frc.robot.commands.manipulator.CommandManipulator;
import frc.robot.commands.manipulator.GoToPositionManipulator;
import frc.robot.commands.scoring.ScoreFromNumpad;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraType;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;

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
                        Constants.ArmConstants.PITCH_FLOOR_OFFSET, Constants.ArmConstants.POWER_LIMIT_PIVOT,
                        Constants.ArmConstants.POWER_LIMIT_EXTEND, new Constraints(0, 0));
        private final TurretSubsystem turretSubsystem = new TurretSubsystem(31);
        private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(43, 44);

        private final CameraSubsystem cameraSubsystem = new CameraSubsystem(CameraType.LIMELIGHT);
        private final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();

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
                                () -> -driverJoytick.getRawAxis(OIConstants.kYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kXAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

                /* Arm Controls (Base) */
                armSubsystem.setDefaultCommand(
                                new JoystickArmMagic(armSubsystem,
                                                () -> towerJoytick.getRawAxis(Constants.OIConstants.kTowerExtendAxis),
                                                () -> towerJoytick.getRawAxis(Constants.OIConstants.kTowerPivotAxis)
                                                                / 2));

                manipulatorSubsystem.setDefaultCommand(new JoystickManipulator(manipulatorSubsystem,
                                () -> towerJoytick.getRawAxis(Constants.OIConstants.kTowerManipulatorLeftAxis),
                                () -> towerJoytick.getRawAxis(Constants.OIConstants.kTowerManipulatorRightAxis)));

                // new JoystickButton(driverJoytick, Constants.OIConstants.kButtonLeftBumper)
                // .onTrue(new ScoreFromNumpad(
                // scoringSubsystem,
                // armSubsystem,
                // turretSubsystem));

                new JoystickButton(driverJoytick, Constants.OIConstants.kButtonLeftBumper).onTrue(
                                new PivotToDegreeMagic(126, // 78
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 2,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> true,
                                                armSubsystem));

                new JoystickButton(driverJoytick, Constants.OIConstants.kButtonRightBumper).onTrue(
                                new PivotToDegreeMagic(90, // 78
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 3,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> true,
                                                armSubsystem));

                /* Extend Controls (Base) */

                new JoystickButton(driverJoytick, Constants.OIConstants.kButtonA)
                                .onTrue(new GoToPositionManipulator(Constants.ManipulatorConstants.CUBE_LEFT + 3,
                                                Constants.ManipulatorConstants.CUBE_RIGHT + 3, manipulatorSubsystem))
                                .debounce(0.25)
                                .onTrue(new GoToPositionManipulator(Constants.ManipulatorConstants.CONE_LEFT
                                                + 5,
                                                Constants.ManipulatorConstants.CONE_RIGHT + 5, manipulatorSubsystem));

                new JoystickButton(driverJoytick, Constants.OIConstants.kButtonY)
                                .onTrue(new KnockConeLeftStageOne(armSubsystem, manipulatorSubsystem, turretSubsystem))
                                .onFalse(new KnockConeLeftStageTwo(armSubsystem, manipulatorSubsystem,
                                                turretSubsystem));

                new JoystickButton(driverJoytick, Constants.OIConstants.kButtonX)
                                .onTrue(new KnockConeRightStageOne(armSubsystem, manipulatorSubsystem, turretSubsystem))
                                .onFalse(new KnockConeRightStageTwo(armSubsystem, manipulatorSubsystem,
                                                turretSubsystem));

                new JoystickButton(driverJoytick, Constants.OIConstants.kButtonB)
                                .onTrue(new CommandManipulator(.2, 15, 0.7, true,
                                                manipulatorSubsystem));

                new JoystickButton(towerJoytick, Constants.OIConstants.kButtonY)
                                .onTrue(new ExtendTicks(60, armSubsystem));
                new JoystickButton(towerJoytick, Constants.OIConstants.kButtonX)
                                .onTrue(new ExtendTicks(35, armSubsystem));

                new JoystickButton(towerJoytick, Constants.OIConstants.kButtonB)
                                .onTrue(new ExtendTicks(0, armSubsystem));

                new JoystickButton(driverJoytick, Constants.OIConstants.kButtonStart)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                // new JoystickButton(driverJoytick, Constants.OIConstants.kButtonX)
                // .whileTrue(new AprilTagMoverCommand(swerveSubsystem, cameraSubsystem,
                // driverJoytick));

                /* Turret Controls (Base) */
                new POVButton(driverJoytick, 90).onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 90, 160));

                new POVButton(driverJoytick, 180)
                                .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 90, 180));

                new POVButton(driverJoytick, 270)
                                .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 90, 200));

                new POVButton(driverJoytick, 0)
                                .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 100, 0));
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
