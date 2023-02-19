// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.TurretSubsystem;

public class RotateToDegree extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean isFinished = false;
  private double desiredAngle;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(TurretSubsystem turretSubsystem, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.desiredAngle = desiredAngle;

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = this.turretSubsystem.getTurretAngle();
    double achieveableAngle = MathUtil.clamp(desiredAngle, -180, 180);
    double angleDifference = currentAngle - achieveableAngle;
    double powerSign = Math.signum(angleDifference);

    if (Math.abs(angleDifference) <= 5) {
      isFinished = true;
    } else {
      this.turretSubsystem.powerTurret(powerSign);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
