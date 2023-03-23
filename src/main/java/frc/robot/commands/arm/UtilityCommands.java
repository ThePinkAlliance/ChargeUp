// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;



import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

import frc.robot.commands.StowReversedExtend;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.extend.ExtendTicksPlus;
import frc.robot.commands.arm.grabber.CommandGrabber;
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
    return new PivotToDegreeMagicNew(angle, // 78
        Constants.ArmConstants.MAX_CRUISE_VELOCITY,
        Constants.ArmConstants.MAX_ACCELERATION, 3,
        Constants.ArmConstants.MOTIONM_GAINS_FX,
        () -> true,
        armSubsystem);
  }

  public static Command scoreCubeHighAuto(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return UtilityCommands.pivotArm(125, armSubsystem).alongWith(new ExtendTicksPlus(82, 0.5, extenderSubsystem))
        .andThen(new GrabberOpen(grabberSubsystem, 1))
        .andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command deliverCubeHigh(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return UtilityCommands.pivotArm(125, armSubsystem).alongWith(new ExtendTicksPlus(82, 0.5, extenderSubsystem));
  }

  public static Command deliverCubeMid(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem,
      ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return UtilityCommands.pivotArm(125, armSubsystem).alongWith(new ExtendTicksPlus(22, 0.5, extenderSubsystem));
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

  public static Command deliverConeMid(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem) {
    return UtilityCommands.pivotArm(127, armSubsystem).alongWith(new ExtendTicks(35, extenderSubsystem));
  }

  public static Command deliverConeHighAuto(ArmSubsystem armSubsystem, 
  ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem, GrabberSubsystem grabberSubsystem) {
return UtilityCommands.pivotArm(130, armSubsystem).alongWith(new ExtendTicks(107, extenderSubsystem)).andThen(new GrabberOpen(grabberSubsystem, 1));
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
