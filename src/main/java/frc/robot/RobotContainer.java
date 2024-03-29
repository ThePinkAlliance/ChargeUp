package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.JoystickArmExtend;
import frc.robot.commands.arm.UtilityCommands;
import frc.robot.commands.arm.grabber.CommandGrabberTerminateCurrent;
import frc.robot.commands.arm.grabber.GrabberOpen;
import frc.robot.commands.arm.grabber.JoystickGrabber;
import frc.robot.commands.arm.turret.JoystickTurret;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.commands.drive.DriveStraightByGyro;
import frc.robot.commands.drive.StrafeByGyro;
import frc.robot.commands.drive.SwerveJoystickCmd;

import frc.robot.commands.drive.autos.DockAuto;

import frc.robot.commands.drive.autos.ScoreHighCenterAndLeaveCommunity;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraType;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.subsystems.arm.GrabberSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;
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

                swerveSubsystem.calibrate();

                configureControllerBindings();

                SmartDashboard.putData("Auto Chooser", autoSendable);

                debugTab.add(xController);
                debugTab.add(yController);
                debugTab.add(thetaController);

                try {
                        /**
                         * These are all testing trajectories for pathweaver.
                         */
                        autoSendable.addOption("Score Cone High, Leave Community Right Balance",
                                        UtilityCommands
                                                        .deliverConeHighAuto(armSubsystem, extenderSubsystem,
                                                                        turretSubsystem, grabberSubsystem)
                                                        .andThen(UtilityCommands
                                                                        .stow(armSubsystem, turretSubsystem,
                                                                                        extenderSubsystem))
                                                        .andThen(new DriveStraightByGyro(-4.35, 3.3,
                                                                        swerveSubsystem)
                                                                        .andThen(new StrafeByGyro(2.2,
                                                                                        3,
                                                                                        swerveSubsystem))
                                                                        .andThen(new DockAuto(
                                                                                        swerveSubsystem,
                                                                                        0,
                                                                                        2, 37, 3.0))));

                        autoSendable.addOption("Drive Auto", new DriveStraightByGyro(-4.35, 3.3,
                                        swerveSubsystem));

                        autoSendable.addOption("Score Cone High, Leave Community Left Balance",
                                        UtilityCommands
                                                        .deliverConeHighAuto(armSubsystem, extenderSubsystem,
                                                                        turretSubsystem, grabberSubsystem)
                                                        .andThen(UtilityCommands
                                                                        .stow(armSubsystem, turretSubsystem,
                                                                                        extenderSubsystem))
                                                        .andThen(new DriveStraightByGyro(-4.35, 3.3,
                                                                        swerveSubsystem)
                                                                        .andThen(new StrafeByGyro(-2.2,
                                                                                        3,
                                                                                        swerveSubsystem))
                                                                        .andThen(new DockAuto(
                                                                                        swerveSubsystem,
                                                                                        0,
                                                                                        2, 37, 3.0))));

                        autoSendable.addOption("Score Cube High, Leave Community Center Balance",
                                        UtilityCommands
                                                        .scoreCubeHighAuto(extenderSubsystem, turretSubsystem,
                                                                        armSubsystem, grabberSubsystem,
                                                                        swerveSubsystem)
                                                        .andThen(ScoreHighCenterAndLeaveCommunity
                                                                        .leaveCommunityCenter(swerveSubsystem,
                                                                                        armSubsystem))

                                                        .andThen(ScoreHighCenterAndLeaveCommunity
                                                                        .balanceStation(swerveSubsystem)));

                        autoSendable.addOption("Auto Center Score", UtilityCommands.scoreCubeHighAuto(extenderSubsystem,
                                        turretSubsystem, armSubsystem, grabberSubsystem, swerveSubsystem));

                        autoSendable.addOption("Auto Balence", ScoreHighCenterAndLeaveCommunity
                                        .leaveCommunityCenter(swerveSubsystem,
                                                        armSubsystem)

                                        .andThen(ScoreHighCenterAndLeaveCommunity
                                                        .balanceStation(swerveSubsystem)));

                        autoSendable.addOption("Score Cube Test", UtilityCommands.scoreCubeHighAuto(extenderSubsystem,
                                        turretSubsystem, armSubsystem, grabberSubsystem, swerveSubsystem));

                        autoSendable.addOption("Score Cone High, Leave Community Left or Right",
                                        UtilityCommands
                                                        .deliverConeHighAuto(armSubsystem, extenderSubsystem,
                                                                        turretSubsystem, grabberSubsystem)
                                                        .andThen(
                                                                        new DriveStraightByGyro(-4.35, 3,
                                                                                        swerveSubsystem)));

                        autoSendable.addOption("Test",
                                        UtilityCommands
                                                        .scoreCubeHighAuto(extenderSubsystem, turretSubsystem,
                                                                        armSubsystem, grabberSubsystem,
                                                                        swerveSubsystem));

                        autoSendable.addOption("Do Nothing", new InstantCommand());
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
                                () -> driverJoystick.getRawAxis(3) > 0.05));

                turretSubsystem.setDefaultCommand(
                                new JoystickTurret(turretSubsystem, () -> towerJoystick.getRawAxis(
                                                OIConstants.kXAxis) * 0.6, armSubsystem));

                /* Tower Default Commands for Pivot, Extend and Turret */
                extenderSubsystem.setDefaultCommand(new JoystickArmExtend(towerJoystick, extenderSubsystem,
                                () -> towerJoystick.getRawAxis(Constants.OIConstants.kTowerExtendAxis)));

                grabberSubsystem.setDefaultCommand(new JoystickGrabber(
                                () -> towerJoystick.getRawAxis(Constants.OIConstants.kTowerPivotAxis),
                                () -> 0.0,
                                grabberSubsystem));

                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonStart)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                // Base Left Bumper - Collect cone station.
                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonLeftBumper).onTrue(
                                UtilityCommands.collectStationDeployCone(armSubsystem, turretSubsystem,
                                                grabberSubsystem, extenderSubsystem))
                                .onFalse(UtilityCommands.collectStationStowCone(armSubsystem, turretSubsystem,
                                                grabberSubsystem, extenderSubsystem));

                // Base Right Bumper - Pickup off ground.
                new JoystickButton(driverJoystick, Constants.OIConstants.kButtonRightBumper)
                                .onTrue(UtilityCommands.pivotArm(83.5 - 1.5, armSubsystem).alongWith(
                                                new CommandGrabberTerminateCurrent(-.7, -2, grabberSubsystem)
                                                                .customCurrentLimit(10).customWatchdog(
                                                                                3)))

                                .onFalse(new CommandGrabberTerminateCurrent(-.7, -16, grabberSubsystem)
                                                .customCurrentLimit(10).customWatchdog(
                                                                3)
                                                .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem,
                                                                extenderSubsystem)));

                // Tower Right Bumper - Score Low
                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonRightBumper)
                                .onTrue(UtilityCommands.pivotArm(125, armSubsystem));

                // Tower Left Trigger - Turret Rotate 180
                new Trigger(() -> towerJoystick.getRawAxis(2) > 0.05)
                                .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 120, 180));

                // Tower Right Trigger - Turret Rotate 0
                new Trigger(() -> towerJoystick.getRawAxis(3) > 0.05)
                                .onTrue(new RotateToDegree(turretSubsystem, armSubsystem, 120, 0));

                // Tower Y - deliver cone high
                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonY).onTrue(
                                UtilityCommands.deliverConeHigh(armSubsystem, extenderSubsystem));

                // Tower B - deliver cone mid
                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonB).onTrue(
                                UtilityCommands.deliverConeMid(armSubsystem, extenderSubsystem));

                // Tower X - stow
                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonX).onTrue(
                                UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));

                // Tower A - release
                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonA).onTrue(
                                new GrabberOpen(grabberSubsystem,
                                                Constants.GrabberConstants.GRABBER_GRASP_OPEN_POWER).powerIntake(0.2));

                // Tower Bumpers
                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonLeftBumper)
                                .onTrue(UtilityCommands.deliverCubeHigh(
                                                extenderSubsystem, turretSubsystem, armSubsystem, grabberSubsystem,
                                                swerveSubsystem));

                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonRightBumper)
                                .onTrue(UtilityCommands.deliverCubeMid(
                                                extenderSubsystem, turretSubsystem, armSubsystem, grabberSubsystem,
                                                swerveSubsystem));
        }

        public void onDisabledInit() {
                armSubsystem.setPositionToHold(0);
                armSubsystem.getPivotTalon().configFactoryDefault();
        }

        public void configureTele() {
                swerveSubsystem.setGyroHeading(180);
                swerveSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        }

        public Command getAutonomousCommand() {
                /**
                 * Reset the turret encoder to the robot's legal position.
                 */
                swerveSubsystem.resetOdometry(new Pose2d());
                swerveSubsystem.zeroHeading();

                return autoSendable.getSelected();
        }
}
