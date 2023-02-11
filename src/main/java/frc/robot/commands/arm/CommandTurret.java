// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.TurretSubsystem;

public class CommandTurret extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private Supplier<Double> rotSupplier;

  /** Creates a new CommandTurret. */
  public CommandTurret(TurretSubsystem turretSubsystem, Supplier<Double> rotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.turretSubsystem = turretSubsystem;
    this.rotSupplier = rotSupplier;

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.turretSubsystem.commandTurret(rotSupplier.get());
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
