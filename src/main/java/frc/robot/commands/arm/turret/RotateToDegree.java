// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.TurretSubsystem;

/**
 * Rotate command for the turret. Right now its configured with a dead reckon
 * controller however it is planned to switch over to pid.
 */
public class RotateToDegree extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean isFinished;
  private double desiredAngle;
  private double lastAngle;
  private double startingAngle;
  private Supplier<Double> pivotSupplier;
  private Timer timer;
  private Timer epoch;
  private PIDController controller;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(TurretSubsystem turretSubsystem, double desiredAngle, Supplier<Double> pivotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.desiredAngle = desiredAngle;
    this.lastAngle = 1;
    this.pivotSupplier = pivotSupplier;
    this.timer = new Timer();
    this.epoch = new Timer();
    this.controller = new PIDController(2.3, 0.0121, 0);
    this.controller.disableContinuousInput();

    SmartDashboard.putNumber("turret-kP", controller.getP());
    SmartDashboard.putNumber("turret-kI", controller.getI());
    SmartDashboard.putNumber("turret-kD", controller.getD());

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();

    epoch.start();
    epoch.reset();

    isFinished = false;
    startingAngle = turretSubsystem.getTurretAngle();

    double kP = SmartDashboard.getNumber("turret-kP", controller.getP());
    double kI = SmartDashboard.getNumber("turret-kI", controller.getI());
    double kD = SmartDashboard.getNumber("turret-kD", controller.getD());

    controller.setPID(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle = this.turretSubsystem.getTurretAngle() * (Math.PI / 180);
    double desiredPosRadians = desiredAngle * (Math.PI / 180);
    double controlEffort = controller.calculate(currentAngle, desiredPosRadians);

    turretSubsystem.powerTurret(controlEffort);

    SmartDashboard.putNumber("Turret Target", desiredPosRadians);
    SmartDashboard.putNumber("Turret Position", currentAngle);

    epoch.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
