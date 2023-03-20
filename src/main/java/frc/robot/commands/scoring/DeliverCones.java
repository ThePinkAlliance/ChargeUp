// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.UtilityCommands;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.extend.ExtendTicksPlus;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;

/** Add your docs here. */
public class DeliverCones {
  public static Command deliverHigh(ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem) {
    return new SequentialCommandGroup(UtilityCommands.pivotArm(132, armSubsystem),
        new ExtendTicksPlus(99, extenderSubsystem));
  }

  public static Command deliverMid(ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem) {
    return new SequentialCommandGroup(UtilityCommands.pivotArm(130, armSubsystem),
        new ExtendTicks(31, extenderSubsystem));
  }
}
