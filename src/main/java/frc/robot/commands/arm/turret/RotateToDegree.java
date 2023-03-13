// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Telmetery;
import frc.robot.subsystems.arm.TurretSubsystem;

/**
 * Rotate command for the turret. Right now its configured with a dead reckon
 * controller however it is planned to switch over to pid.
 */
public class RotateToDegree extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean isFinished;
  private double desiredAngle;
  private Supplier<Boolean> safeToContinue;
  private CANSparkMax sparkMax;
  private double angleTolerence;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(TurretSubsystem turretSubsystem, double desiredAngle, Supplier<Boolean> safeToContinue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.desiredAngle = desiredAngle;
    this.safeToContinue = safeToContinue;
    this.angleTolerence = .3;

    this.sparkMax = turretSubsystem.getCanSparkMax();

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    sparkMax.getPIDController().setP(0.1);
    sparkMax.getPIDController().setI(0);
    sparkMax.getPIDController().setD(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle = this.turretSubsystem.getTurretAngle() * (Math.PI / 180);
    double desiredPosRadians = desiredAngle * (Math.PI / 180);
    double desiredRotations = desiredPosRadians * (348.7 / (2 * Math.PI));

    if (safeToContinue.get()) {
      sparkMax.getPIDController().setReference(desiredRotations, ControlType.kPosition);
    } else {
      isFinished = true;
    }

    Telmetery.logData("Turret Target Angle", desiredPosRadians, RotateToDegree.class);
    Telmetery.logData("Current Turret Angle", currentAngle, RotateToDegree.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDesired = this.turretSubsystem.getTurretAngle() * (Math.PI / 180);
    double difference = Math.abs(desiredAngle - currentDesired);

    return difference <= angleTolerence || isFinished;
  }
}
