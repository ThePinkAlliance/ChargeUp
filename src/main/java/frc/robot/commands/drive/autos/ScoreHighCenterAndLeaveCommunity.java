// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.UtilityCommands;
import frc.robot.commands.arm.extend.ExtendTicksPlus;
import frc.robot.commands.arm.grabber.GrabberOpen;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.subsystems.arm.GrabberSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

/** Add your docs here. */
public class ScoreHighCenterAndLeaveCommunity {
  public static Command scoreCubeHigh(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
    return UtilityCommands.pivotArm(127, armSubsystem).alongWith(new ExtendTicksPlus(86, 0.5, extenderSubsystem)).andThen(new GrabberOpen(grabberSubsystem, 1)).andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  // public Command getCommand(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
  //   return scoreCubeHigh(extenderSubsystem, turretSubsystem, null, null)
  // }
}
