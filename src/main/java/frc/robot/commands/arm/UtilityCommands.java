// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

import frc.robot.commands.StowReversedExtendNoTurret;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.grabber.CommandGrabberTerminateCurrent;
import frc.robot.commands.arm.grabber.GrabberOpen;
import frc.robot.commands.arm.pivot.PivotToDegreeMagicNew;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.subsystems.arm.GrabberSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

/** Add your docs here. */
public class UtilityCommands {

  public static Command pivotArm(double angle, ArmSubsystem armSubsystem) {
    if (angle < 72) {
      throw new UnknownError("Invalid pitch angle");
    }

    return new PivotToDegreeMagicNew(angle,
        Constants.ArmConstants.MAX_CRUISE_VELOCITY,
        Constants.ArmConstants.MAX_ACCELERATION, 3,
        Constants.ArmConstants.MOTIONM_GAINS_FX,
        () -> true,
        armSubsystem);
  }

  public static Command scoreCubeHighAuto(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return new RotateToDegree(turretSubsystem, armSubsystem, 90, 0)
        .andThen(UtilityCommands.pivotArm(125, armSubsystem).alongWith(new ExtendTicks(82, extenderSubsystem))
            .andThen(
                new GrabberOpen(grabberSubsystem, Constants.GrabberConstants.GRABBER_GRASP_OPEN_POWER).powerIntake(-.3))
            .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem)));
  }

  public static Command deliverCubeHigh(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return UtilityCommands.pivotArm(127, armSubsystem).alongWith(new ExtendTicks(82, extenderSubsystem));
  }

  public static Command deliverCubeMid(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return UtilityCommands.pivotArm(127, armSubsystem).alongWith(new ExtendTicks(20, extenderSubsystem));
  }

  public static Command collectStationDeployCone(ArmSubsystem armSubsystem,
      TurretSubsystem turretSubsystem,
      GrabberSubsystem grabberSubsystem, ExtenderSubsystem extenderSubsystem) {
    return UtilityCommands.pivotArm(145, armSubsystem).alongWith(
        new CommandGrabberTerminateCurrent(-Constants.GrabberConstants.GRABBER_GRASP_CLOSE_POWER, 0, grabberSubsystem)
            .customWatchdog(10).customCurrentLimit(22).noKill());
  }

  public static Command collectStationStowCone(ArmSubsystem armSubsystem,
      TurretSubsystem turretSubsystem,
      GrabberSubsystem grabberSubsystem, ExtenderSubsystem extenderSubsystem) {
    return new CommandGrabberTerminateCurrent(-.7, -16, grabberSubsystem).customCurrentLimit(14).customWatchdog(3)
        .andThen(new StowReversedExtendNoTurret(armSubsystem, extenderSubsystem));
  }

  public static Command deliverConeHigh(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem) {
    // Comp: 132.5, 107
    return UtilityCommands.pivotArm(129, armSubsystem).alongWith(new ExtendTicks(108, extenderSubsystem));
  }

  public static Command deliverConeMid(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem) {
    return UtilityCommands.pivotArm(129.5, armSubsystem).alongWith(new ExtendTicks(31, extenderSubsystem));
  }

  public static Command deliverConeHighAuto(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem, GrabberSubsystem grabberSubsystem) {
    return new RotateToDegree(turretSubsystem, armSubsystem, 90, 0).andThen(
        UtilityCommands.pivotArm(130, armSubsystem).alongWith(
            new ExtendTicks(108, extenderSubsystem)))
        .andThen(new GrabberOpen(grabberSubsystem,
            Constants.GrabberConstants.GRABBER_GRASP_OPEN_POWER))
        .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command deliverConeHighAutoBlue(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem, GrabberSubsystem grabberSubsystem) {
    return new RotateToDegree(turretSubsystem, armSubsystem, 90, -5).andThen(
        UtilityCommands.pivotArm(130, armSubsystem).alongWith(
            new ExtendTicks(108, extenderSubsystem)))
        .andThen(new GrabberOpen(grabberSubsystem,
            Constants.GrabberConstants.GRABBER_GRASP_OPEN_POWER))
        .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command deliverConeHighAutoRed(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem, GrabberSubsystem grabberSubsystem) {
    return new RotateToDegree(turretSubsystem, armSubsystem, 90, 0).andThen(
        UtilityCommands.pivotArm(130, armSubsystem).alongWith(
            new ExtendTicks(108, extenderSubsystem)))
        .andThen(new GrabberOpen(grabberSubsystem,
            Constants.GrabberConstants.GRABBER_GRASP_OPEN_POWER))
        .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command stow(ArmSubsystem armSubsystem, TurretSubsystem turretSubsystem,
      ExtenderSubsystem extenderSubsystem) {
    return new ParallelCommandGroup(pivotArm(
        Constants.ArmConstants.COLLECT_STOW,
        armSubsystem),
        new ExtendTicks(0,
            extenderSubsystem))
        .andThen(new RotateToDegree(turretSubsystem, armSubsystem, 90, 0));
  }

  public static Command stowPitch(ArmSubsystem armSubsystem) {
    return pivotArm(Constants.ArmConstants.COLLECT_STOW, armSubsystem);
  }
}
