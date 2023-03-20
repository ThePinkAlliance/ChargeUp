// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;
import java.util.function.Supplier;
import com.ThePinkAlliance.core.util.GainsFX;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.ArmSubsystem;

public class PivotToDegreeMagicNew extends CommandBase {
  private final double angleFactor;

  private int smoothingIntensity;
  private double cruiseVelocity;
  private double acceleration;
  private double desiredAngle;
  private ArmSubsystem armSubsystem;
  private TalonFX pivotMotor;
  private Supplier<Boolean> safeToContinue;

  private double initialAngle;

  private boolean isFinished;
  private Watchdog watchdog;
  private final double WATCHDOG_TIMEOUT = 1.5;
  

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreeMagicNew(double desiredAngle, Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desiredAngle = desiredAngle;
    this.safeToContinue = safeToContinue;
    this.armSubsystem = armSubsystem;
    this.pivotMotor = armSubsystem.getPivotTalon();
    this.angleFactor = 0.000612669993242;
    this.smoothingIntensity = 0;
    this.acceleration = 2000;
    this.cruiseVelocity = 2040;
    this.isFinished = false;
    this.initialAngle = 0;
    this.watchdog = new Watchdog(WATCHDOG_TIMEOUT, () -> {
      // empty on purpose, end() will handle safing the subsystem
    });

    addRequirements(armSubsystem);
  }

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreeMagicNew(double desiredAngle, double cruiseVelocity, double acceleration, int smoothingIntensity,
      GainsFX gains,
      Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
     this(desiredAngle, safeToContinue, armSubsystem);
    this.acceleration = acceleration;
    this.cruiseVelocity = cruiseVelocity;
    this.smoothingIntensity = smoothingIntensity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
    armSubsystem.configureTalonFX_MotionMagic(cruiseVelocity, acceleration, smoothingIntensity);
    initialAngle = armSubsystem.getArmPitch();
    pivotMotor.setSelectedSensorPosition(0);
    double desiredPosition = (desiredAngle - initialAngle) / angleFactor;
    pivotMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);
    watchdog.reset();
    watchdog.enable();
    System.out.println("---- Pivot Init ----");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = pivotMotor.getSelectedSensorPosition();
    double currentPitch = ((currentPosition + .001) / 1578.6776859504) + initialAngle;
    boolean isSafe = safeToContinue.get();

    //pivotMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);

    Telemetry.logData("isSafe", isSafe, PivotToDegreeMagicNew.class);
    Telemetry.logData("isFinished", isFinished, PivotToDegreeMagicNew.class);
    Telemetry.logData("currentAngle", currentPitch, PivotToDegreeMagicNew.class);
    Telemetry.logData("desiredAngle", desiredAngle, PivotToDegreeMagicNew.class);
    Telemetry.logData("initalAngle", initialAngle, PivotToDegreeMagicNew.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.setPositionToHold(pivotMotor.getSelectedSensorPosition());
    System.out.println("---- Pivot Command Terminated ----");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition = pivotMotor.getSelectedSensorPosition();
    double desiredPosition = (desiredAngle - initialAngle) / angleFactor;
    double diff = Math.abs(desiredPosition - currentPosition);
    boolean differenceMet = diff <= 139; // 29.5794701987;
    boolean watchdogKill = watchdog.isExpired();

    Telemetry.logData("Pitch Difference", diff, PivotToDegreeMagicNew.class);
    Telemetry.logData("Current Pitch Position", currentPosition, PivotToDegreeMagicNew.class);
    Telemetry.logData("Desired Pitch Position", desiredPosition, PivotToDegreeMagicNew.class);

    if (watchdogKill) {
      Telemetry.logData("----- WatchDog Kill; Pos Difference -----", diff, PivotToDegreeMagicNew.class);
    }

    if (differenceMet) {
      Telemetry.logData("----- Difference Met; Pos Difference -----", diff, PivotToDegreeMagicNew.class);
    }

    return (differenceMet || watchdogKill);
  }
}
