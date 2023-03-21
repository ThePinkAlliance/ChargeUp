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
  private double intakeSpeed;
  private double graspRotations;
  private Watchdog watchdog;

  public CommandGrabber(double intakeSpeed, double graspRotations, GrabberSubsystem grabberSubsystem) {
    this.intakeSpeed = intakeSpeed;
    this.graspRotations = graspRotations;
    this.grabberSubsystem = grabberSubsystem;

    this.watchdog = new Watchdog(3, () -> {
      this.grabberSubsystem.setIntakeSpeed(0);
      this.grabberSubsystem.disableGrasp();
    });

    addRequirements(grabberSubsystem);
  }

  @Override
  public void initialize() {
    this.grabberSubsystem.setIntakeSpeed(intakeSpeed);
    this.grabberSubsystem.setGraspRotations(graspRotations);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    this.grabberSubsystem.setIntakeSpeed(0);
    this.grabberSubsystem.disableGrasp();
  }

  @Override
  public boolean isFinished() {
    return grabberSubsystem.intakeAtCurrentLimit() || watchdog.isExpired();
  }
}
