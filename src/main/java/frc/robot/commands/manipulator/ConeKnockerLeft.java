// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class ConeKnockerLeft extends CommandBase {
  ManipulatorSubsystem manipulatorSubsystem;
  double startingPosLeft;
  double startingPosRight;
  double endPosLeft;
  double endPosRight;
  boolean leftReachedEnd;
  boolean rightReachedEnd;
  boolean isClamped;

  /** Creates a new ConeKnockerLeft. */
  public ConeKnockerLeft(double startingPosLeft, double startingPosRight, double endPosLeft, double endPosRight,
      ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.manipulatorSubsystem = manipulatorSubsystem;
    this.startingPosLeft = startingPosLeft;
    this.startingPosRight = startingPosRight;
    this.endPosLeft = endPosLeft;
    this.endPosRight = endPosRight;

    this.rightReachedEnd = false;
    this.leftReachedEnd = false;
    this.isClamped = false;

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightReachedEnd = false;
    leftReachedEnd = false;

    this.manipulatorSubsystem.setPositionTargetRight(startingPosRight);
    this.manipulatorSubsystem.setPositionTargetRight(startingPosLeft);
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
    return false;
  }
}
