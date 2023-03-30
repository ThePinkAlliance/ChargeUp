// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.GrabberSubsystem;

public class GrabberOpen extends CommandBase {
  private GrabberSubsystem grabberSubsystem;
  private Watchdog watchdog;
  private Timer sustainedCurrentDrawTimer;
  private boolean isFinished;
  private double openPower;
  private double intakePower;

  /** Creates a new GrabberGrasp. */
  public GrabberOpen(GrabberSubsystem grabberSubsystem, double openPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.grabberSubsystem = grabberSubsystem;
    this.isFinished = false;
    this.intakePower = 0;
    this.watchdog = new Watchdog(Constants.GrabberConstants.GRABBER_GRASP_OPEN_WATCHDOG, () -> {
      this.grabberSubsystem.setGraspPower(0);
    });
    this.sustainedCurrentDrawTimer = new Timer();
    this.openPower = openPower;
    addRequirements(grabberSubsystem);
  }

  public GrabberOpen powerIntake(double power) {
    this.intakePower = power;

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.grabberSubsystem.setGraspRotations(targetRotations);
    sustainedCurrentDrawTimer.stop();
    sustainedCurrentDrawTimer.reset();
    watchdog.reset();
    watchdog.enable();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabberSubsystem.setGraspPower(openPower);
    grabberSubsystem.setIntakeSpeed(intakePower);
    if (grabberSubsystem
        .graspAtCustomCurrentLimit(Constants.GrabberConstants.GRABBER_GRASP_SUSTAINED_CURRENT_OPEN_LIMIT)) {
      sustainedCurrentDrawTimer.start();
      if (sustainedCurrentDrawTimer.hasElapsed(Constants.GrabberConstants.GRABBER_GRASP_SUSTAINED_CURRENT_OPEN_TIMEOUT))
        isFinished = true;
    } else {
      sustainedCurrentDrawTimer.stop();
      sustainedCurrentDrawTimer.reset();
    }
    System.out.println("Grabber.Grasp.Current = " + grabberSubsystem.getGraspCurrent());
    System.out.println("Grabber.Grasp.Position = " + grabberSubsystem.getGraspRotations());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.grabberSubsystem.disableGrasp();
    watchdog.disable();
    sustainedCurrentDrawTimer.stop();
    grabberSubsystem.zeroGraspPosition();
    grabberSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean watchDogTripped = watchdog.isExpired();
    System.out.println("GrabberOpen.isFinished = " + isFinished);
    System.out.println("GrabberOpen.watchDog   = " + watchDogTripped);
    return isFinished || watchDogTripped;
  }
}
