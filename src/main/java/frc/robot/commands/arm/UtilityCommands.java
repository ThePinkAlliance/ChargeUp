// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.StowReversedExtend;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.extend.ExtendTicksPlus;
import frc.robot.commands.arm.grabber.CommandGrabber;
import frc.robot.commands.arm.grabber.GrabberCollect;
import frc.robot.commands.arm.grabber.GrabberOpen;
import frc.robot.commands.arm.pivot.PivotToDegreeMagicNew;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.commands.manipulator.CommandManipulator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.subsystems.arm.GrabberSubsystem;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

/** Add your docs here. */
public class UtilityCommands {
  public static Command zeroManipulator(ManipulatorSubsystem manipulatorSubsystem) {
    return new CommandManipulator(.2, 15, 0.7, true,
        manipulatorSubsystem);
  }

  public static Command pivotArm(double angle, ArmSubsystem armSubsystem) {
    return new PivotToDegreeMagicNew(angle, // 78
        Constants.ArmConstants.MAX_CRUISE_VELOCITY,
        Constants.ArmConstants.MAX_ACCELERATION, 3,
        Constants.ArmConstants.MOTIONM_GAINS_FX,
        () -> true,
        armSubsystem);
  }

  public static Command scoreCubeHigh(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return UtilityCommands.pivotArm(127, armSubsystem).alongWith(new ExtendTicksPlus(86, 0.5, extenderSubsystem))
        .andThen(new GrabberOpen(grabberSubsystem, 1))
        .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command scoreConeHigh(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return new PivotToDegreeMagicNew(131, // 78
        Constants.ArmConstants.MAX_CRUISE_VELOCITY,
        Constants.ArmConstants.MAX_ACCELERATION - 500, 3,
        Constants.ArmConstants.MOTIONM_GAINS_FX,
        () -> true,
        armSubsystem).alongWith(new ExtendTicksPlus(106, 0.5, extenderSubsystem))
        .andThen(new GrabberOpen(grabberSubsystem, 1))
        .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command collectStationDeployCone(ArmSubsystem armSubsystem,
      TurretSubsystem turretSubsystem,
      GrabberSubsystem grabberSubsystem, ExtenderSubsystem extenderSubsystem) {
    return UtilityCommands.pivotArm(148, armSubsystem).alongWith(
        new CommandGrabber(-Constants.GrabberConstants.GRABBER_GRASP_CLOSE_POWER, 0, grabberSubsystem));
  }

  public static Command collectStationStowCone(ArmSubsystem armSubsystem,
      TurretSubsystem turretSubsystem,
      GrabberSubsystem grabberSubsystem, ExtenderSubsystem extenderSubsystem) {
    return new CommandGrabber(0, -20, grabberSubsystem)
        .andThen(new StowReversedExtend(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command deliverConeHigh(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem) {
    return UtilityCommands.pivotArm(130, armSubsystem).alongWith(new ExtendTicks(107, extenderSubsystem));
  }

  public static Command stow(ArmSubsystem armSubsystem, TurretSubsystem turretSubsystem,
      ExtenderSubsystem extenderSubsystem) {
    return new ExtendTicks(0, extenderSubsystem)
        .alongWith(new PivotToDegreeMagicNew(Constants.ArmConstants.COLLECT_STOW, // 78
            Constants.ArmConstants.MAX_CRUISE_VELOCITY,
            Constants.ArmConstants.MAX_ACCELERATION, 3,
            Constants.ArmConstants.MOTIONM_GAINS_FX,
            () -> true,
            armSubsystem))
        .andThen(new RotateToDegree(turretSubsystem, armSubsystem, 90, 0));
  }

}
