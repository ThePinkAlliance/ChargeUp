// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.grabber;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.GrabberSubsystem;

/** Add your docs here. */
public class CommandGrabber extends CommandBase {
  private GrabberSubsystem grabberSubsystem;
  private double collectSpeed;
  private double graspRotations;
  private Watchdog watchdog;

  public CommandGrabber(double collectSpeed, double graspRotations, GrabberSubsystem grabberSubsystem) {
    this.collectSpeed = collectSpeed;
    this.graspRotations = graspRotations;
    this.grabberSubsystem = grabberSubsystem;

    this.watchdog = new Watchdog(3, () -> {
      this.grabberSubsystem.setCollectSpeed(0);
      this.grabberSubsystem.disableGrasp();
    });

    addRequirements(grabberSubsystem);
  }

  @Override
  public void initialize() {
    this.grabberSubsystem.setCollectSpeed(collectSpeed);
    this.grabberSubsystem.setGraspRotations(graspRotations);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    this.grabberSubsystem.setCollectSpeed(0);
    this.grabberSubsystem.disableGrasp();
  }

  @Override
  public boolean isFinished() {
    return grabberSubsystem.collectAtCurrentLimit() || watchdog.isExpired();
  }
}
