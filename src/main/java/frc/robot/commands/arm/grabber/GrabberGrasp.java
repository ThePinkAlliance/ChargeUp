// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.GrabberSubsystem;

public class GrabberGrasp extends CommandBase {
  private GrabberSubsystem grabberSubsystem;
  private double targetRotations;

  /** Creates a new GrabberGrasp. */
  public GrabberGrasp(double targetRotations, GrabberSubsystem grabberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.grabberSubsystem = grabberSubsystem;
    this.targetRotations = targetRotations;

    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.grabberSubsystem.setGraspRotations(targetRotations);
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
    return true;
  }
}
