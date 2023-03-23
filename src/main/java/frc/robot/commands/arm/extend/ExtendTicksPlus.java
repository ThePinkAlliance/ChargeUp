// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.extend;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.Telemetry;

public class ExtendTicksPlus extends CommandBase {
  ExtenderSubsystem extenderSubsystem;
  double desiredRotations;
  Watchdog watchdog;
  private final double WATCHDOG_TIMEOUT = 3.0;
  private double lastMin;
  private double lastMax;
  private double outputRange;

  /** Creates a new ExtendTicks. */
  public ExtendTicksPlus(double desiredRotations, ExtenderSubsystem extenderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extenderSubsystem = extenderSubsystem;
    this.desiredRotations = desiredRotations;
    this.outputRange = 0;
    this.watchdog = new Watchdog(WATCHDOG_TIMEOUT, () -> {
      this.extenderSubsystem.commandExtend(0);
    });

    addRequirements(extenderSubsystem);
  }

  public ExtendTicksPlus(double desiredRotations, double outputRange, ExtenderSubsystem extenderSubsystem) {
    this(desiredRotations, extenderSubsystem);

    this.outputRange = outputRange;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    watchdog.reset();
    watchdog.enable();

    if (outputRange != 0) {
      this.lastMin = this.extenderSubsystem.getOutputMin();
      this.lastMax = this.extenderSubsystem.getOutputMax();

      this.extenderSubsystem.configureOutputRange(outputRange);
    }

    extenderSubsystem.disableExtendForwardSoftLimits();
    extenderSubsystem.disableExtendReverseSoftLimits();
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
    extenderSubsystem.enableExtendForwardSoftLimits();
    extenderSubsystem.enableExtendReverseSoftLimits();
    extenderSubsystem.configureOutputRange(lastMin, lastMax);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extenderSubsystem.atExtensionSetpoint() || watchdog.isExpired();
  }
}
