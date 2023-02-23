// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class PIDTunerPivot extends CommandBase {
  ArmSubsystem armSubsystem;

  /** Creates a new PIDTunerPivot. */
  public PIDTunerPivot(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controlEffort = this.armSubsystem.calculatePivotInput(180);

    SmartDashboard.putNumber("pivot Angle", armSubsystem.getPivotAngle());
    SmartDashboard.putNumber("control effort", controlEffort);
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
