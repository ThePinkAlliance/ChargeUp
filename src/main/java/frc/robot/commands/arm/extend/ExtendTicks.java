// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.extend;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.Telemetry;

public class ExtendTicks extends CommandBase {
  ArmSubsystem armSubsystem;
  double desiredRotations;
  Watchdog watchdog;
  private final double WATCHDOG_TIMEOUT = 3.0;

  /** Creates a new ExtendTicks. */
  public ExtendTicks(double desiredRotations, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.desiredRotations = desiredRotations;
    this.watchdog = new Watchdog(WATCHDOG_TIMEOUT, () -> {
      this.armSubsystem.commandExtend(0);
    });

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    watchdog.reset();
    watchdog.enable();
    armSubsystem.setExtenionRotations(desiredRotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Telemetry.logData("Ticks", armSubsystem.getExtensionRotations(), ExtendTicks.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atExtensionSetpoint() || watchdog.isExpired();
  }
}
