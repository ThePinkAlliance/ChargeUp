// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class CommandCurrentManipulator extends CommandBase {
  ManipulatorSubsystem manipulatorSubsystem;
  double desiredCurrent;
  MedianFilter leftFilter;
  MedianFilter rightFilter;
  boolean leftLocked;
  boolean rightLocked;
  Timer leftEngaged;
  Timer rightEngaged;

  /** Creates a new CloseManipulator. */
  public CommandCurrentManipulator(double desiredCurrent, ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.desiredCurrent = desiredCurrent;
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.leftFilter = new MedianFilter(20);
    this.rightFilter = new MedianFilter(20);
    this.leftLocked = false;
    this.rightLocked = false;

    this.rightEngaged = new Timer();
    this.leftEngaged = new Timer();

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.leftLocked = false;
    this.rightLocked = false;

    this.manipulatorSubsystem.setLeftPower(0.2);
    this.manipulatorSubsystem.setRightPower(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPos = leftFilter.calculate(this.manipulatorSubsystem.getLeftPosition());
    double rightPos = rightFilter.calculate(this.manipulatorSubsystem.getRightPosition());

    if (Math.abs(this.manipulatorSubsystem.getLeftVelocity()) < 10) {
      double position = this.manipulatorSubsystem.getLeftPosition() + 0.5;

      this.manipulatorSubsystem.setLeftControl(position, ControlType.kPosition);
    }

    if (Math.abs(this.manipulatorSubsystem.getRightVelocity()) < 10) {
      double position = this.manipulatorSubsystem.getRightPosition() + 0.5;

      this.manipulatorSubsystem.setLeftControl(position, ControlType.kPosition);
    }

    // if (leftCurrent >= 24.8) {
    // if (leftEngaged.hasElapsed(1)) {
    // double position = this.manipulatorSubsystem.getLeftPosition() + 4;

    // this.manipulatorSubsystem.setLeftControl(position, ControlType.kPosition);
    // this.leftLocked = true;
    // }
    // leftEngaged.start();
    // } else {
    // leftEngaged.reset();
    // leftEngaged.stop();
    // }

    // if (rightCurrent >= 24.8) {
    // if (rightEngaged.hasElapsed(1)) {
    // double position = this.manipulatorSubsystem.getRightPosition() + 4;

    // this.manipulatorSubsystem.setRightControl(position, ControlType.kPosition);
    // this.rightLocked = true;
    // }
    // rightEngaged.start();
    // } else {
    // rightEngaged.reset();
    // rightEngaged.stop();
    // }
    System.out.println("l: " + leftPos + ", r: " + rightPos);
  }

  // Called once the command ends or is interrupted.
  //
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double diffL = Math.abs(desiredCurrent -
    // this.manipulatorSubsystem.getLeftCurrent());
    // double diffR = Math.abs(desiredCurrent -
    // this.manipulatorSubsystem.getRightCurrent());

    return this.leftLocked && this.rightLocked;
  }
}
