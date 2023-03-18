// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.StowReveredExtend;
import frc.robot.commands.arm.UtilityCommands;
import frc.robot.commands.arm.pivot.PivotToDegreeMagic;
import frc.robot.commands.manipulator.GoToPositionManipulator;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupFromGroundCone {
  public static Command stageOne(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    return new ParallelCommandGroup(new PivotToDegreeMagic(81.5, // 78
        Constants.ArmConstants.MAX_CRUISE_VELOCITY,
        Constants.ArmConstants.MAX_ACCELERATION, 3,
        Constants.ArmConstants.MOTIONM_GAINS_FX,
        () -> true,
        armSubsystem), UtilityCommands.zeroManipulator(manipulatorSubsystem));

  }

  public static Command stageTwo(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem,
      TurretSubsystem turretSubsystem) {
    return new SequentialCommandGroup(new GoToPositionManipulator(
        Constants.ManipulatorConstants.CONE_LEFT
            + Constants.ManipulatorConstants.CONE_GRIP_MULTIPLER,
        Constants.ManipulatorConstants.CONE_RIGHT
            + Constants.ManipulatorConstants.CONE_GRIP_MULTIPLER,
        manipulatorSubsystem), new StowReveredExtend(armSubsystem, turretSubsystem));
  }
}
