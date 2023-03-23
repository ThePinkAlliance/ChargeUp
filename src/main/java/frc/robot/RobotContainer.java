package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.StowReversedExtend;
import frc.robot.commands.arm.JoystickArm;
import frc.robot.commands.arm.JoystickArmExtend;
import frc.robot.commands.arm.UtilityCommands;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.extend.ExtendTicksPlus;
import frc.robot.commands.arm.grabber.CommandGrabber;
import frc.robot.commands.arm.grabber.GrabberCollect;
import frc.robot.commands.arm.grabber.GrabberOpen;
import frc.robot.commands.arm.grabber.JoystickGrabber;
import frc.robot.commands.arm.pivot.PivotToDegreeMagicNew;
import frc.robot.commands.arm.turret.JoystickTurret;
import frc.robot.commands.arm.turret.RotateBasedOnExternalSensor;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.commands.drive.DriveStraightByGyro;
import frc.robot.commands.drive.Navigate;
import frc.robot.commands.drive.StrafeByGyro;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.drive.TimedNavigate;
import frc.robot.commands.drive.autos.DockAuto;

import frc.robot.commands.drive.autos.ScoreHighCenterAndLeaveCommunity;
import frc.robot.commands.drive.autos.TAutoScoreCubeLeftLeave;
import frc.robot.commands.manipulator.CommandManipulator;
import frc.robot.commands.manipulator.GoToPositionManipulator;
import frc.robot.commands.scoring.DeliverCones;
import frc.robot.commands.scoring.PickupFromGroundCone;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraType;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.subsystems.arm.GrabberSubsystem;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;
import frc.robot.subsystems.camera.CameraInterface.PipelineType;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final ShuffleboardTab debugTab = Shuffleboard.getTab("debug");
        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final Joystick towerJoystick = new Joystick(OIConstants.kTowerControllerPort);

        private SendableChooser<Command> autoSendable = new SendableChooser<Command>();

        public static PIDController xController = new PIDController(AutoConstants.kPXController, 0.5, 0);
        public static PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        public static ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

        private final ArmSubsystem armSubsystem = new ArmSubsystem(41, 9,
                        Constants.ArmConstants.PITCH_FLOOR_OFFSET, Constants.ArmConstants.POWER_LIMIT_PIVOT);
        private final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem(42,
                        Constants.ExtenderConstants.POWER_LIMIT_EXTEND);
        private final TurretSubsystem turretSubsystem = new TurretSubsystem(31);

        private final CameraSubsystem cameraSubsystem = new CameraSubsystem(CameraType.LIMELIGHT);
        private final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();
        private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

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

                        Command timedScoreOne = new TimedNavigate(swerveSubsystem, new ChassisSpeeds(-1, 0, 0), .5);
                        // Command leaveCommunity = new Navigate(swerveSubsystem, new
                        // SwerveModulePosition(
                        // 2.89, new Rotation2d()), 1.2)
                        // .alongWith(new CommandManipulator(.2, 15, 0.7, true,
                        // manipulatorSubsystem));

                        autoSendable.addOption("Strafe Left", new StrafeByGyro(2, 3, swerveSubsystem).debug());
                        autoSendable.addOption("Strafe Right", new StrafeByGyro(-2, 3, swerveSubsystem).debug());
                        autoSendable.addOption("Leave Community Right",
                                        UtilityCommands
                                                        .scoreConeHigh(extenderSubsystem, turretSubsystem, armSubsystem,
                                                                        grabberSubsystem, swerveSubsystem)
                                                        .andThen(new DriveStraightByGyro(-4.35, 3, swerveSubsystem)
                                                                        .andThen(new StrafeByGyro(-1.7, 2,
                                                                                        swerveSubsystem))
                                                                        .andThen(new DockAuto(swerveSubsystem, 0,
                                                                                        2, 37, 3.0))));

                        autoSendable.addOption("Leave Community Left",
                                        new DriveStraightByGyro(-4.35, 3, swerveSubsystem)
                                                        .andThen(new StrafeByGyro(-1.7, 2, swerveSubsystem))
                                                        .andThen(new DockAuto(swerveSubsystem, 0,
                                                                        2, 37, 3.0)));

                        autoSendable.addOption("Do Nothing", new InstantCommand());
                        autoSendable.addOption("Score One",
                                        timedScoreOne);
                        autoSendable.addOption("Center, Score Cube, Balence, and Leave Community",
                                        UtilityCommands
                                                        .scoreCubeHigh(extenderSubsystem, turretSubsystem, armSubsystem,
                                                                        grabberSubsystem, swerveSubsystem)
                                                        .andThen(ScoreHighCenterAndLeaveCommunity
                                                                        .leaveCommunityCenter(swerveSubsystem,
                                                                                        armSubsystem))
                                                        .andThen(ScoreHighCenterAndLeaveCommunity
                                                                        .balanceStation(swerveSubsystem)));
                } catch (Exception err) {
                        err.printStackTrace();
                }
        }

        private void configureControllerBindings() {
                /* Drivetrain (Base) */
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoystick.getRawAxis(OIConstants.kYAxis),
                                () -> -driverJoystick.getRawAxis(OIConstants.kXAxis),
                                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

                turretSubsystem.setDefaultCommand(
                                new JoystickTurret(turretSubsystem, () -> towerJoystick.getRawAxis(0), armSubsystem));

                /* Tower Default Commands for Pivot, Extend and Turret */
                extenderSubsystem.setDefaultCommand(new JoystickArmExtend(towerJoystick, extenderSubsystem,
                                () -> towerJoystick.getRawAxis(Constants.OIConstants.kTowerExtendAxis)));

                grabberSubsystem.setDefaultCommand(new JoystickGrabber(
                                () -> towerJoystick.getRawAxis(Constants.OIConstants.kTowerPivotAxis),
                                () -> towerJoystick.getRawAxis(Constants.OIConstants.lTowerTurretAxis),
                                grabberSubsystem));

                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonB).onTrue(UtilityCommands
                                .pivotArm(127, armSubsystem).alongWith(new ExtendTicksPlus(96, 0.5, extenderSubsystem)))
                                .onFalse(new GrabberCollect(
                                                Constants.GrabberConstants.GRABBER_GRASP_CLOSE_POWER, grabberSubsystem)
                                                .andThen(
                                                                UtilityCommands.stow(armSubsystem, turretSubsystem,
                                                                                extenderSubsystem)));

                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonLeftBumper).onTrue(
                                new PivotToDegreeMagicNew(132,
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 2,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> true,
                                                armSubsystem));

                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonRightBumper).onTrue(
                                new PivotToDegreeMagicNew(82,
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 2,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> true,
                                                armSubsystem));

                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonBack).onTrue(new InstantCommand(
                                () -> swerveSubsystem.setModuleStates(Constants.DriveConstants.kDriveKinematics
                                                .toSwerveModuleStates(new ChassisSpeeds()))));

                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonB)
                                .onTrue(new GrabberOpen(grabberSubsystem,
                                                Constants.GrabberConstants.GRABBER_GRASP_OPEN_POWER));

                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonY).onTrue(
                                UtilityCommands.deliverConeHigh(armSubsystem, extenderSubsystem));

                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonX).onTrue(UtilityCommands
                                .collectStationDeployCone(armSubsystem, turretSubsystem, grabberSubsystem,
                                                extenderSubsystem))
                                .onFalse(UtilityCommands.collectStationStowCone(armSubsystem, turretSubsystem,
                                                grabberSubsystem, extenderSubsystem));

                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonX)
                                .onTrue(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
                new Trigger(() -> towerJoystick.getRawAxis(2) > 0.05)
                                .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 120, 180));

                new Trigger(() -> towerJoystick.getRawAxis(3) > 0.05)
                                .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 120, 0));

                // new POVButton(towerJoystick, 0)
                // .onTrue(DeliverCones.deliverHigh(armSubsystem, extenderSubsystem));
                // new POVButton(towerJoystick, 270)
                // .onTrue(DeliverCones.deliverMid(armSubsystem, extenderSubsystem));

                // new POVButton(towerJoystick, 180)
                // .onTrue(new ExtendTicks(0, extenderSubsystem));

                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonStart)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                /* Turret Controls (Base) */
                // new POVButton(driverJoystick, 270).onTrue(new RotateToDegree(turretSubsystem,
                // armSubsystem, 90,
                // 160 + Constants.TurretConstants.TURRET_OFFSET));

                // new POVButton(driverJoystick, 180).onTrue(new RotateToDegree(turretSubsystem,
                // armSubsystem, 90, 180));

                // new POVButton(driverJoystick, 90)
                // .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 90, 200
                // - Constants.TurretConstants.TURRET_OFFSET));

                // new POVButton(driverJoystick, 0)
                // .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 100, 0));
        }

        public void onDisabledInit() {
                armSubsystem.setPositionToHold(0);
                armSubsystem.getPivotTalon().configFactoryDefault();
        }

        public Command getAutonomousCommand() {
                return autoSendable.getSelected();
                // Trajectory trajectory = autoSendable.getSelected();

                // // Reset the swerve subsystem pose to the inital pose of the trajectory.
                // swerveSubsystem.resetOdometry(trajectory.getInitialPose());

                // // 4. Construct command to follow trajectory
                // Command swerveControllerCommand =
                // swerveSubsystem.buildSwerveCommand(trajectory);

                // // 5. Add some init and wrap-up, and return everything
                // return new SequentialCommandGroup(
                // new InstantCommand(() -> SmartDashboard.putString("InitalPose",
                // swerveSubsystem.getPose()
                // .toString()),
                // swerveSubsystem),
                // swerveControllerCommand.alongWith(),
                // new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));
        }
}
