// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class ZeroManipulator extends CommandBase {
  ManipulatorSubsystem manipulatorSubsystem;
  MedianFilter leftFilter;
  MedianFilter rightFilter;
  boolean isFinished;
  boolean leftZeroed;
  boolean rightZeroed;
  Timer leftSustainedTime;
  Timer rightSustainedTime;
  double sustainedTime = .4;
  double threshold = 23;

  /** Creates a new ZeroManipulator. */
  public ZeroManipulator(ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.manipulatorSubsystem = manipulatorSubsystem;
    this.leftFilter = new MedianFilter(40);
    this.rightFilter = new MedianFilter(40);
    this.isFinished = false;
    this.leftZeroed = false;
    this.rightZeroed = false;

    this.rightSustainedTime = new Timer();
    this.leftSustainedTime = new Timer();

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
    this.leftZeroed = false;
    this.rightZeroed = false;

    leftSustainedTime.reset();
    rightSustainedTime.reset();

    manipulatorSubsystem.setLeftPower(-0.6);
    manipulatorSubsystem.setRightPower(-0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lC = manipulatorSubsystem.getLeftCurrent();
    double rC = manipulatorSubsystem.getLeftCurrent();

    double leftCurrent = leftFilter.calculate(lC);
    double rightCurrent = rightFilter.calculate(rC);

    if (leftCurrent >= threshold) {
      leftSustainedTime.start();
    } else {
      leftSustainedTime.stop();
      leftSustainedTime.reset();
    }

    if (rightCurrent >= threshold) {
      rightSustainedTime.start();
    } else {
      rightSustainedTime.stop();
      rightSustainedTime.reset();
    }

    if (leftSustainedTime.hasElapsed(sustainedTime)) {
      this.manipulatorSubsystem.setLeftPower(0);
      this.leftZeroed = true;
    }

    if (rightSustainedTime.hasElapsed(sustainedTime)) {
      this.manipulatorSubsystem.setRightPower(0);
      this.rightZeroed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.leftFilter.reset();
    this.rightFilter.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftZeroed && rightZeroed;
  }
}
