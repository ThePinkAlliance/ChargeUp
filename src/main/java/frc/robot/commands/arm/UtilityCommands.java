// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.StowReversedExtend;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.extend.ExtendTicksPlus;
import frc.robot.commands.arm.pivot.PivotToDegreeMagicNew;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.commands.manipulator.CommandManipulator;
import frc.robot.commands.manipulator.GoToPositionManipulator;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
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

 public static Command collectHighDeploy(ArmSubsystem armSubsystem,
      TurretSubsystem turretSubsystem,
      ManipulatorSubsystem manipulatorSubsystem, ExtenderSubsystem extenderSubsystem) {
    return new CommandManipulator(.2, 15, 0.7, true,
        manipulatorSubsystem)
        .alongWith(UtilityCommands.pivotArm(128, armSubsystem).andThen(new ExtendTicksPlus(60, extenderSubsystem)));
  }

  public static Command collectHighStow(ArmSubsystem armSubsystem,
      TurretSubsystem turretSubsystem,
      ManipulatorSubsystem manipulatorSubsystem, ExtenderSubsystem extenderSubsystem) {
    return new GoToPositionManipulator(
        Constants.ManipulatorConstants.CONE_LEFT
            + Constants.ManipulatorConstants.CONE_GRIP_MULTIPLER,
        Constants.ManipulatorConstants.CONE_RIGHT + Constants.ManipulatorConstants.CONE_GRIP_MULTIPLER,
        manipulatorSubsystem)
        .andThen(new StowReversedExtend(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command deliverConeHigh(ArmSubsystem armSubsystem,
      ExtenderSubsystem extenderSubsystem
      ) {
    return UtilityCommands.pivotArm(130, armSubsystem).alongWith(new ExtendTicksPlus(85, extenderSubsystem));
  }

  public static Command stow(ArmSubsystem armSubsystem, TurretSubsystem turretSubsystem,
      ExtenderSubsystem extenderSubsystem
      ) {
    return new ExtendTicks(0, extenderSubsystem).alongWith(new PivotToDegreeMagicNew(Constants.ArmConstants.COLLECT_STOW, // 78
    Constants.ArmConstants.MAX_CRUISE_VELOCITY,
    Constants.ArmConstants.MAX_ACCELERATION, 3,
    Constants.ArmConstants.MOTIONM_GAINS_FX,
    () -> true,
    armSubsystem).alongWith(new RotateToDegree(turretSubsystem, armSubsystem, 90, 0)));
  }
  

}
