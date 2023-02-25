// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class JoystickManipulator extends CommandBase {
  ManipulatorSubsystem manipulator;
  Supplier<Double> leftSupplier;
  Supplier<Double> rightSupplier;

  /** Creates a new CommandManipulator. */
  public JoystickManipulator(ManipulatorSubsystem manipulator, Supplier<Double> leftSupplier,
      Supplier<Double> rightSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.manipulator = manipulator;
    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;

    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftCurrent = manipulator.getLeftCurrent();
    double rightCurrent = manipulator.getRightCurrent();

    double leftPosition = manipulator.getLeftPosition();
    double rightPosition = manipulator.getRightPosition();

    SmartDashboard.putNumber(this.getName() + " leftCurrent", leftCurrent);
    SmartDashboard.putNumber(this.getName() + " rightCurrent", rightCurrent);

    SmartDashboard.putNumber(this.getName() + " leftPosition", leftPosition);
    SmartDashboard.putNumber(this.getName() + " rightPosition", rightPosition);

    manipulator.setRightPower(rightSupplier.get());
    manipulator.setLeftPower(leftSupplier.get());
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
