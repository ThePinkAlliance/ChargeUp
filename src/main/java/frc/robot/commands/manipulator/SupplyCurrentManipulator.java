// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ManipulatorSubsystem;

public class SupplyCurrentManipulator extends CommandBase {
  double pwr;
  ManipulatorSubsystem manipulatorSubsystem;

  /** Creates a new SupplyCurrentManipulator. */
  public SupplyCurrentManipulator(double pwr, ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.pwr = pwr;

    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.manipulatorSubsystem.setLeftPower(0.2);
    this.manipulatorSubsystem.setRightPower(0.2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out
        .println(this.manipulatorSubsystem.getLeftVelocity() + ", " + this.manipulatorSubsystem.getRightVelocity());
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
