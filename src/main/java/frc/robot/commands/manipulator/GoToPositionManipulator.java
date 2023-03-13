// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;
import frc.robot.Telemetry;

public class GoToPositionManipulator extends CommandBase {
  ManipulatorSubsystem manipulatorSubsystem;
  double desiredPositionL, desiredPositionR;
  Timer timer;
  Watchdog watchdog;
  double tolerance;

  /** Creates a new CloseManipulator. */
  public GoToPositionManipulator(double desiredPositionL,
      double desiredPositionR, ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.desiredPositionR = desiredPositionR;

    this.desiredPositionL = desiredPositionL;
    this.timer = new Timer();
    this.tolerance = .4;
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.watchdog = new Watchdog(1.75, () -> {
      this.manipulatorSubsystem.setLeftPower(0);
      this.manipulatorSubsystem.setRightPower(0);
    });

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!Double.isNaN(desiredPositionL)) {
      this.manipulatorSubsystem.setPositionTargetLeft(desiredPositionL);
    }

    if (!Double.isNaN(desiredPositionR)) {
      this.manipulatorSubsystem.setPositionTargetRight(desiredPositionR);
    }

    watchdog.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Telemetry.logData("Manipulator Right", this.manipulatorSubsystem.getRightPosition(), GoToPositionManipulator.class);
    Telemetry.logData("Manipulator Left", this.manipulatorSubsystem.getLeftPosition(), GoToPositionManipulator.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.manipulatorSubsystem.setLeftPower(0);
    this.manipulatorSubsystem.setRightPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double diffL = Math.abs(desiredPositionL - this.manipulatorSubsystem.getLeftPosition());
    double diffR = Math.abs(desiredPositionR - this.manipulatorSubsystem.getRightPosition());

    if (Double.isNaN(desiredPositionL)) {
      diffL = 0;
    }

    if (Double.isNaN(desiredPositionR)) {
      diffR = 0;
    }

    return diffL < tolerance && diffR < tolerance || watchdog.isExpired();
  }
}
