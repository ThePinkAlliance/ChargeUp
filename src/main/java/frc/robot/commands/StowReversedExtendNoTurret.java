// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.extend.ExtendTicks;
import frc.robot.commands.arm.pivot.PivotToDegreeMagicNew;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowReversedExtendNoTurret extends SequentialCommandGroup {
  /** Creates a new Stow. */
  public StowReversedExtendNoTurret(ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PivotToDegreeMagicNew(Constants.ArmConstants.COLLECT_STOW, // 78
        Constants.ArmConstants.MAX_CRUISE_VELOCITY,
        Constants.ArmConstants.MAX_ACCELERATION, 3,
        Constants.ArmConstants.MOTIONM_GAINS_FX,
        () -> true, armSubsystem), new ExtendTicks(0, extenderSubsystem));
  }
}
