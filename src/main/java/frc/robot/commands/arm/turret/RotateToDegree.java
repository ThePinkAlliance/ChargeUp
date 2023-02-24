// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.TurretSubsystem;

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
    // double vel = (Math.abs(currentAngle) - Math.abs(st)) / epoch.get();

    // TrapezoidProfile.State currentState = new State(currentAngle, vel);
    // TrapezoidProfile.State goalState = new State(desiredAngle, 0);

    // TrapezoidProfile profile = new TrapezoidProfile(new Constraints(30, 15),
    // goalState, currentState);
    // TrapezoidProfile.State t = profile.calculate(timer.get());
    // double power = t.velocity * (1 / 36.6);

    // if (profile.isFinished(timer.get())) {
    // isFinished = true;
    // }

    // SmartDashboard.putNumber("profile pos", t.position);
    // SmartDashboard.putNumber("profile vel", t.velocity);
    // this.turretSubsystem.powerTurretUnsafe(power);

    if (Math.abs(angleDifference) < 1) {
      isFinished = true;
    }
    this.turretSubsystem.powerTurret(powerSign);

    // SmartDashboard.putNumber("vel", vel);
    SmartDashboard.putNumber("currentAngle", currentAngle);
    SmartDashboard.putNumber("angleDiff", angleDifference);
    // SmartDashboard.putNumber("power", power);
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
