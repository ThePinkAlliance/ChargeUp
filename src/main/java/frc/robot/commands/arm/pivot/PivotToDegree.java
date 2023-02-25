// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import java.util.List;

import com.ThePinkAlliance.core.math.LinearInterpolationTable;
import com.ThePinkAlliance.core.math.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class PivotToDegree extends CommandBase {
  ArmSubsystem armSubsystem;
  boolean isFinished;
  LinearInterpolationTable table = new LinearInterpolationTable(List.of(new Vector2d(80, 0.07), new Vector2d(86, 0.07),
      new Vector2d(98, 0.05), new Vector2d(107, 0.07), new Vector2d(117, 0.0708), new Vector2d(122, 0.015),
      new Vector2d(133, 0.066), new Vector2d(140, 0.031), new Vector2d(148, 0.070), new Vector2d(157, 0.051),
      new Vector2d(167, 0.041), new Vector2d(178, 0.051), new Vector2d(190, 0.011)));
  double desiredAngle;

  /** Creates a new PivotToDegree. */
  public PivotToDegree(ArmSubsystem armSubsystem, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
    this.isFinished = false;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = armSubsystem.getPivotAngle();
    double diff = Math.abs(desiredAngle - currentAngle);
    double ff = table.interp(currentAngle);
    double directon = Math.signum(desiredAngle - currentAngle) * 0.5;

    if (Double.isNaN(ff) || Double.isInfinite(ff)) {
      ff = 0;
    }

    if (diff < 2) {
      isFinished = true;
    }

    this.armSubsystem.commandPivotUnsafe(ff + directon);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
