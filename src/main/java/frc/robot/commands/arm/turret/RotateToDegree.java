// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.Telemetry;

/**
 * Rotate command for the turret. Right now its configured with a dead reckon
 * controller however it is planned to switch over to pid.
 */
public class RotateToDegree extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean isFinished;
  private double desiredAngle;
  private double safetyPivotAngle;
  private CANSparkMax sparkMax;
  private ArmSubsystem armSubsystem;
  private double angleTolerence;
  private Watchdog watchdog;

  // Revert to 2.5 if rotate issue not fixed.
  private final double WATCHDOG_TIMEOUT = 2.5;
  private boolean resetMotor;
  private double resetPosition;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem, double safetyPivotAngle,
      double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
    this.resetMotor = false;
    this.resetPosition = 0;
    this.angleTolerence = .05;
    this.safetyPivotAngle = safetyPivotAngle;
    this.watchdog = new Watchdog(WATCHDOG_TIMEOUT, () -> {
      // empty on purpose, end() will handle safing the subsystem
    });
    this.sparkMax = turretSubsystem.getCanSparkMax();

    // Do not require armSubsystem: its only here to get information
    addRequirements(turretSubsystem);
  }

  public RotateToDegree withReset(double resetPosition) {
    this.resetMotor = true;
    this.resetPosition = resetPosition;

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    sparkMax.getPIDController().setP(0.1);
    sparkMax.getPIDController().setI(0);
    sparkMax.getPIDController().setD(0);
    watchdog.reset();

    if (resetMotor) {
      this.turretSubsystem.setEncoderPositions(resetPosition);
    }

    double currentAngle = this.turretSubsystem.getTurretAngle() * (Math.PI / 180);
    double desiredPosRadians = desiredAngle * (Math.PI / 180);
    double desiredRotations = desiredPosRadians * (Constants.TurretConstants.FULL_MOTOR_ROTATIONS / (2 * Math.PI));
    System.out.println("armSubSystem.getArmPitch() " + armSubsystem.getArmPitch());

    if (armSubsystem.getArmPitch() > safetyPivotAngle) {
      sparkMax.getPIDController().setReference(desiredRotations, ControlType.kPosition);
    } else {
      isFinished = true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armSubsystem.getArmPitch() < safetyPivotAngle) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);

    this.watchdog.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDesired = this.turretSubsystem.getTurretAngle();
    double difference = Math.abs(desiredAngle - currentDesired);

    boolean watchdogExpired = watchdog.isExpired();
    boolean hasMetTarget = difference <= angleTolerence;

    if (hasMetTarget) {
      Telemetry.logData("--- Rotate To Degree Terminated [met target] ---", "difference: " + difference, getClass());
    }

    if (watchdogExpired) {
      Telemetry.logData("--- Rotate To Degree [watchdog] ---", "difference: " + difference, getClass());
    }

    if (isFinished) {
      Telemetry.logData("--- Rotate To Degree ---", "difference: " + difference, getClass());
    }

    return hasMetTarget || isFinished || watchdogExpired;
  }
}
