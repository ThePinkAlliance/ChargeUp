// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.extend;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.Telemetry;

public class ExtendTicks extends CommandBase {
  ExtenderSubsystem extenderSubsystem;
  double desiredRotations;
  Watchdog watchdog;
  private final double WATCHDOG_TIMEOUT = 2.0;

  /** Creates a new ExtendTicks. */
  public ExtendTicks(double desiredRotations, ExtenderSubsystem extenderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extenderSubsystem = extenderSubsystem;
    this.desiredRotations = desiredRotations;
    this.watchdog = new Watchdog(WATCHDOG_TIMEOUT, () -> {
      this.extenderSubsystem.commandExtend(0);
    });

    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    watchdog.reset();
    watchdog.enable();
    extenderSubsystem.setExtenionRotations(desiredRotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Telemetry.logData("Ticks", extenderSubsystem.getExtensionRotations(), ExtendTicks.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extenderSubsystem.atExtensionSetpoint() || watchdog.isExpired();
  }
}
