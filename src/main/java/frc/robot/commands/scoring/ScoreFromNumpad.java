// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.commands.arm.pivot.PivotToDegreeMagic;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreFromNumpad extends SequentialCommandGroup {
    /** Creates a new ScoreFromNumpad. */
    public ScoreFromNumpad(ScoringSubsystem scoringSubsystem, ArmSubsystem armSubsystem,
            TurretSubsystem turretSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(
                new PivotToDegreeMagic(scoringSubsystem.getPositionData_Pitch(),
                        Constants.ArmConstants.MAX_CRUISE_VELOCITY,
                        Constants.ArmConstants.MAX_ACCELERATION, 3,
                        Constants.ArmConstants.MOTIONM_GAINS_FX,
                        () -> false,
                        armSubsystem),
                new RotateToDegree(turretSubsystem, armSubsystem, 80, scoringSubsystem.getPositionData_Turret(),
                        () -> 0.0),
                new InstantCommand(() -> {
                    Telemetry.logData("Desired Position Turret", scoringSubsystem.getPositionData_Turret(),
                            ScoreFromNumpad.class);
                    Telemetry.logData("Desired Position Pitch", scoringSubsystem.getPositionData_Pitch(),
                            ScoreFromNumpad.class);
                    Telemetry.logData("Desired Position Extend", scoringSubsystem.getPositionData_Extend(),
                            ScoreFromNumpad.class);
                }));
    }
}
