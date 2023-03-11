// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.pivot.PivotToDegreeMagic;
import frc.robot.commands.manipulator.GoToPositionManipulator;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class KnockConeRightStageTwo extends SequentialCommandGroup {
        /** Creates a new KnockConeLeftStageTwo. */
        public KnockConeRightStageTwo(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem,
                        TurretSubsystem turretSubsystem) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(
                                new GoToPositionManipulator(Double.NaN,
                                                0, manipulatorSubsystem),

                                new GoToPositionManipulator(54, Double.NaN,
                                                manipulatorSubsystem),

                                new PivotToDegreeMagic(128,
                                                Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                                                Constants.ArmConstants.MAX_ACCELERATION, 3,
                                                Constants.ArmConstants.MOTIONM_GAINS_FX,
                                                () -> !turretSubsystem.isMoving(),
                                                armSubsystem),

                                new GoToPositionManipulator(
                                                Constants.ManipulatorConstants.CONE_LEFT
                                                                + 4,
                                                Constants.ManipulatorConstants.CONE_RIGHT
                                                                + 4,
                                                manipulatorSubsystem));
        }
}
