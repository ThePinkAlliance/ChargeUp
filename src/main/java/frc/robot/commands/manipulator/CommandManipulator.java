// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class CommandManipulator extends CommandBase {
  ManipulatorSubsystem manipulatorSubsystem;
  double time;
  boolean inverted;
  MedianFilter leftFilter;
  MedianFilter rightFilter;
  boolean leftZeroed;
  boolean rightZeroed;
  Timer leftSustainedTime;
  Timer rightSustainedTime;
  double sustainedTime = .4;
  double currentThreshold;
  double power;

  /** Creates a new PowerUntilTime. */
  public CommandManipulator(double sustainedTime, double currentThreshold, double power, boolean inverted,
      ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.inverted = inverted;
    this.time = sustainedTime;
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.leftFilter = new MedianFilter(40);
    this.rightFilter = new MedianFilter(40);
    this.leftZeroed = false;
    this.rightZeroed = false;
    this.power = power;
    this.currentThreshold = currentThreshold;
    this.leftSustainedTime = new Timer();
    this.rightSustainedTime = new Timer();

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.leftZeroed = false;
    this.rightZeroed = false;

    leftSustainedTime.stop();
    rightSustainedTime.stop();

    leftSustainedTime.reset();
    rightSustainedTime.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lC = manipulatorSubsystem.getLeftCurrent();
    double rC = manipulatorSubsystem.getLeftCurrent();

    double leftCurrent = leftFilter.calculate(lC);
    double rightCurrent = rightFilter.calculate(rC);

    if (leftCurrent >= currentThreshold) {
      leftSustainedTime.start();
    } else {
      leftSustainedTime.stop();
      leftSustainedTime.reset();
    }

    if (rightCurrent >= currentThreshold) {
      rightSustainedTime.start();
    } else {
      rightSustainedTime.stop();
      rightSustainedTime.reset();
    }

    if (leftSustainedTime.hasElapsed(sustainedTime)) {
      this.manipulatorSubsystem.setLeftPower(0);
      this.leftZeroed = true;
    } else {
      manipulatorSubsystem.setLeftPower(inverted ? -power : power);
    }

    if (rightSustainedTime.hasElapsed(sustainedTime)) {
      this.manipulatorSubsystem.setRightPower(0);
      this.rightZeroed = true;
    } else {
      manipulatorSubsystem.setRightPower(inverted ? -power : power);
    }

    System.out.println(leftCurrent + ", " + rightCurrent);
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
