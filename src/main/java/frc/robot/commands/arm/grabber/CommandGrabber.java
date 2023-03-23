// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.grabber;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.GrabberSubsystem;

/** Add your docs here. */
public class CommandGrabber extends CommandBase {
  private GrabberSubsystem grabberSubsystem;
  private double intakeSpeed;
  private double graspRotations;
  private Watchdog watchdog;

  private boolean doNotKill;

  public CommandGrabber(double intakeSpeed, double graspRotations, GrabberSubsystem grabberSubsystem) {
    this.intakeSpeed = intakeSpeed;
    this.graspRotations = graspRotations;
    this.grabberSubsystem = grabberSubsystem;

    this.watchdog = new Watchdog(2, () -> {
      this.grabberSubsystem.setIntakeSpeed(0);
      this.grabberSubsystem.disableGrasp();
    });

    addRequirements(grabberSubsystem);
  }

  @Override
  public void initialize() {
    this.grabberSubsystem.setIntakeSpeed(intakeSpeed);
    this.grabberSubsystem.setGraspRotations(graspRotations);

    Telemetry.logData("Target Rotations", graspRotations, CommandGrabber.class);

    this.watchdog.reset();
    this.watchdog.enable();

    SmartDashboard.putNumber("Target Rotations Grasp", graspRotations);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    this.grabberSubsystem.setIntakeSpeed(0);
    // this.grabberSubsystem.disableGrasp();

    this.watchdog.disable();
  }

  @Override
  public boolean isFinished() {
    // boolean atCurrentLimit = grabberSubsystem.intakeAtCurrentLimit();
    boolean watchdogKill = watchdog.isExpired();

    // if (atCurrentLimit) {
    // Telemetry.logData("--- Grabber Terminated ---", "at current limited",
    // CommandGrabber.class);
    // }

    SmartDashboard.putNumber("Grabber Diff", graspRotations - grabberSubsystem.getGraspRotations());

    if (watchdogKill) {
      Telemetry.logData("--- Grabber Terminated ---", "watchdog kill", CommandGrabber.class);
    }

    return watchdogKill;
  }
}
