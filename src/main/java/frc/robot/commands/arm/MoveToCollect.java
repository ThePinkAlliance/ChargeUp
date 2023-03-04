// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.pivot.PivotToDegree;
import frc.robot.commands.arm.pivot.PivotToDegreeMagic;
import frc.robot.commands.arm.turret.RotateToDegree;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToCollect extends ParallelCommandGroup {
  /** Creates a new MoveToCollect. */
  public MoveToCollect(TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double targetAngle = 0;
    double targetPitch = 90;
    double minPitch = 110;
    double initalPitch = armSubsystem.getArmPitch();
    double initalAngle = turretSubsystem.getTurretAngle();
    double angleDiff = Math.abs(initalAngle - targetAngle);

    if (angleDiff > 3 && initalPitch < minPitch) {
      addCommands(new RotateToDegree(turretSubsystem, 0, () -> armSubsystem.getArmPitch() > 90), new PivotToDegreeMagic(
          minPitch,
          36864,
          20480, 3, Constants.ArmConstants.MOTIONM_GAINS_FX,
          () -> !turretSubsystem.isMoving(),
          armSubsystem).andThen(
              new PivotToDegreeMagic(
                  targetPitch,
                  36864,
                  20480, 3, Constants.ArmConstants.MOTIONM_GAINS_FX,
                  () -> !turretSubsystem.isMoving(),
                  armSubsystem)));
    } else {

    }

  }
}
