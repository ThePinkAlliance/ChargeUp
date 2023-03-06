// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class ZeroManipulator extends CommandBase {
  ManipulatorSubsystem manipulatorSubsystem;
  MedianFilter leftFilter;
  MedianFilter rightFilter;
  double leftCurrent;
  double rightCurrent;
  boolean isFinished;
  boolean leftZeroed;
  boolean rightZeroed;

  /** Creates a new ZeroManipulator. */
  public ZeroManipulator(ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.manipulatorSubsystem = manipulatorSubsystem;
    this.leftFilter = new MedianFilter(6);
    this.rightFilter = new MedianFilter(6);
    this.isFinished = false;
    this.leftZeroed = false;
    this.rightZeroed = false;

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulatorSubsystem.setLeftPower(-0.4);
    manipulatorSubsystem.setRightPower(-0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lC = manipulatorSubsystem.getLeftCurrent();
    double rC = manipulatorSubsystem.getLeftCurrent();

    leftCurrent = leftFilter.calculate(lC);
    rightCurrent = rightFilter.calculate(rC);

    if (leftCurrent >= 30) {
      manipulatorSubsystem.setLeftPower(0);
      manipulatorSubsystem.resetLeftEncoder();

      this.leftZeroed = true;
    }

    if (rightCurrent >= 30) {
      manipulatorSubsystem.setRightPower(0);
      manipulatorSubsystem.resetRightEncoder();

      this.rightZeroed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftZeroed && rightZeroed;
  }
}
