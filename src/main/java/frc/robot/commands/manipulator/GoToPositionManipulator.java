// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class GoToPositionManipulator extends CommandBase {
  ManipulatorSubsystem manipulatorSubsystem;
  double desiredPosition;

  /** Creates a new CloseManipulator. */
  public GoToPositionManipulator(double desiredPosition, ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.desiredPosition = desiredPosition;
    this.manipulatorSubsystem = manipulatorSubsystem;

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.manipulatorSubsystem.setPositionTargetLeft(desiredPosition);
    this.manipulatorSubsystem.setPositionTargetRight(desiredPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double diffL = Math.abs(desiredPosition - this.manipulatorSubsystem.getLeftPosition());
    double diffR = Math.abs(desiredPosition - this.manipulatorSubsystem.getRightPosition());

    return ((diffL + diffR) / 2) <= 1;
  }
}
