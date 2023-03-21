// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.grabber;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.GrabberSubsystem;

public class GrabberCollect extends CommandBase {
  private Watchdog watchdog;
  private GrabberSubsystem grabberSubsystem;
  private double speed;

  /** Creates a new GrabberCollect. */
  public GrabberCollect(double speed, GrabberSubsystem grabberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.grabberSubsystem = grabberSubsystem;
    this.speed = speed;
    this.watchdog = new Watchdog(3, () -> {
      this.grabberSubsystem.setCollectSpeed(0);
    });

    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.grabberSubsystem.setCollectSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.grabberSubsystem.setCollectSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.grabberSubsystem.collectAtCurrentLimit() || this.watchdog.isExpired();
  }
}
