// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.TurretSubsystem;

/**
 * Rotate command for the turret. Right now its configured with a dead reckon
 * controller however it is planned to switch over to pid.
 */
public class RotateToDegree extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean isFinished;
  private double desiredAngle;
  private double lastAngle;
  private double startingAngle;
  private Timer timer;
  private Timer epoch;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(TurretSubsystem turretSubsystem, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.desiredAngle = desiredAngle;
    this.lastAngle = 1;
    this.timer = new Timer();
    this.epoch = new Timer();

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();

    epoch.start();
    epoch.reset();

    isFinished = false;
    startingAngle = turretSubsystem.getTurretAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = this.turretSubsystem.getTurretAngle();
    double achieveableAngle = MathUtil.clamp(desiredAngle, -180, 180);
    double angleDifference = achieveableAngle - currentAngle;
    double powerSign = Math.signum(angleDifference);

    if (Math.abs(angleDifference) < 1) {
      isFinished = true;
    }
    this.turretSubsystem.powerTurret(powerSign);

    SmartDashboard.putNumber("currentAngle", currentAngle);
    SmartDashboard.putNumber("angleDiff", angleDifference);
    SmartDashboard.putNumber("achieveableAngle", achieveableAngle);

    lastAngle = currentAngle;
    epoch.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
