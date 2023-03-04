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

  /** Creates a new ZeroManipulator. */
  public ZeroManipulator(ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.manipulatorSubsystem = manipulatorSubsystem;
    this.leftFilter = new MedianFilter(6);
    this.rightFilter = new MedianFilter(6);

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulatorSubsystem.setLeftPower(-0.4);
    manipulatorSubsystem.setRightPower(-0.4);

    double lC = manipulatorSubsystem.getLeftCurrent();
    double rC = manipulatorSubsystem.getLeftCurrent();

    leftCurrent = leftFilter.calculate(lC);
    rightCurrent = rightFilter.calculate(rC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.setLeftPower(0);
    manipulatorSubsystem.setRightPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftCurrent >= 30 && rightCurrent >= 30;
  }
}
