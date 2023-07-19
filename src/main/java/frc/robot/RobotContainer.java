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
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.subsystems.arm.GrabberSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final Joystick towerJoystick = new Joystick(OIConstants.kTowerControllerPort);

        public static PIDController xController = new PIDController(AutoConstants.kPXController, 0.5, 0);
        public static PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        public static ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

        private final ArmSubsystem armSubsystem = new ArmSubsystem(41, 9,
                        Constants.ArmConstants.PITCH_FLOOR_OFFSET, Constants.ArmConstants.POWER_LIMIT_PIVOT);
        private final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem(42,
                        Constants.ExtenderConstants.POWER_LIMIT_EXTEND);
        private final TurretSubsystem turretSubsystem = new TurretSubsystem(31);

        private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

        public RobotContainer() {
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                configureControllerBindings();
        }

        private void configureControllerBindings() {
                /* Drivetrain (Base) */

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

                // Tower X - stow
                new JoystickButton(towerJoystick, Constants.OIConstants.kButtonX).onTrue(
                                UtilityCommands.stowPitch(armSubsystem));
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
                return Commands.print("The outreach build does not contain an auto.");
        }
}
