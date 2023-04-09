// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.grabber;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.GrabberSubsystem;

/** Add your docs here. */
public class CommandGrabberTerminateCurrent extends CommandBase {
  private GrabberSubsystem grabberSubsystem;
  private double intakeSpeed;
  private double graspRotations;
  private Watchdog watchdog;

  private MedianFilter medianFilter;
  private boolean noKill;
  private Timer currentTimer;
  private double currentLimit;
  private int filterSize;
  private double currentTime;

  public CommandGrabberTerminateCurrent(double intakeSpeed, double graspRotations, GrabberSubsystem grabberSubsystem) {
    this.intakeSpeed = intakeSpeed;
    this.graspRotations = graspRotations;
    this.grabberSubsystem = grabberSubsystem;
    this.currentTimer = new Timer();
    this.noKill = false;
    this.currentTime = .2;
    this.filterSize = 25;
    this.currentLimit = Constants.GrabberConstants.BETTER_GRABBER_INTAKE_CURRENT_LIMIT;

    this.watchdog = new Watchdog(2, () -> {
      this.grabberSubsystem.setIntakeSpeed(0);
      this.grabberSubsystem.disableGrasp();
    });

    addRequirements(grabberSubsystem);
  }

  public CommandGrabberTerminateCurrent noKill() {
    this.noKill = true;

    return this;
  }

  public CommandGrabberTerminateCurrent customCurrentTime(double time) {
    this.currentTime = time;

    return this;
  }

  public CommandGrabberTerminateCurrent customCurrentLimit(double currentLimit) {
    this.currentLimit = currentLimit;

    return this;
  }

  public CommandGrabberTerminateCurrent customWatchdog(double time) {
    this.watchdog = new Watchdog(time, () -> {
      this.grabberSubsystem.setIntakeSpeed(0);
      this.grabberSubsystem.disableGrasp();
    });

    return this;
  }

  public CommandGrabberTerminateCurrent customFilterSize(int size) {
    this.filterSize = size;

    return this;
  }

  @Override
  public void initialize() {
    this.currentTimer.reset();
    this.medianFilter = new MedianFilter(filterSize);

    this.grabberSubsystem.setIntakeSpeed(intakeSpeed);
    this.grabberSubsystem.setGraspRotations(graspRotations);

    Telemetry.logData("Target Rotations", graspRotations, CommandGrabberTerminateCurrent.class);

    this.watchdog.reset();
    this.watchdog.enable();

    SmartDashboard.putNumber("Target Rotations Grasp", graspRotations);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Intake Current", this.grabberSubsystem.getIntakeCurrent());
  }

  @Override
  public void end(boolean interrupted) {
    if (!noKill) {
      this.grabberSubsystem.setIntakeSpeed(0);
    }
    // this.grabberSubsystem.disableGrasp();

    currentTimer.stop();
    currentTimer.reset();

    this.watchdog.disable();
  }

  @Override
  public boolean isFinished() {
    boolean watchdogKill = watchdog.isExpired();
    double intakeCurrent = medianFilter.calculate(grabberSubsystem.getIntakeCurrent());

    if (intakeCurrent >= currentLimit - 2
        && intakeCurrent <= currentLimit + 2) {
      Telemetry.logData("--- Grabber Terminated ---", "Current at " + intakeCurrent,
          CommandGrabberTerminateCurrent.class);

      currentTimer.start();
    }

    if (currentTimer.hasElapsed(currentTime)) {
      return true;
    }

    SmartDashboard.putNumber("Grabber Diff", graspRotations - grabberSubsystem.getGraspRotations());

    if (watchdogKill) {
      Telemetry.logData("--- Grabber Terminated ---", "watchdog kill", CommandGrabberTerminateCurrent.class);
    }

    return watchdogKill;
  }
}
