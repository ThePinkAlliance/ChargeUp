// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.pivot.PivotToDegreeMagicNew;
import frc.robot.commands.manipulator.CommandManipulator;
import frc.robot.commands.manipulator.GoToPositionManipulator;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class KnockConeRightStageOne extends SequentialCommandGroup {
        /** Creates a new KnockConeLeft. */
        public KnockConeRightStageOne(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem,
                        TurretSubsystem turretSubsystem) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(
                                new PivotToDegreeMagicNew(Constants.ArmConstants.COLLECT_CONE_ANGLE_STAGE_ONE,
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 2,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> !turretSubsystem
                                                                .isMoving(),
                                                armSubsystem).alongWith(
                                                                new CommandManipulator(.2, 15, 0.7, true,
                                                                                manipulatorSubsystem)),

                                new GoToPositionManipulator(Constants.ManipulatorConstants.COLLECT_CONE_FULLY_OPEN,
                                                Constants.ManipulatorConstants.COLLECT_CONE_RIGHT_RIGHT_STAGE_ONE,
                                                manipulatorSubsystem));
        }
}
