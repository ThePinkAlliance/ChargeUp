// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  private CANSparkMax collectMotor;
  private CANSparkMax angleMotor;

  private int collectCurrentLimit;
  private int graspCurrentLimit;

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    this.collectCurrentLimit = 35;
    this.graspCurrentLimit = 40;

    this.angleMotor = new CANSparkMax(43, MotorType.kBrushless);
    this.collectMotor = new CANSparkMax(44, MotorType.kBrushless);

    this.angleMotor.setSmartCurrentLimit(graspCurrentLimit);
    this.collectMotor.setSmartCurrentLimit(collectCurrentLimit);

    this.angleMotor.getPIDController().setP(0.1);
    this.collectMotor.getPIDController().setP(0.1);

    this.collectMotor.setIdleMode(IdleMode.kBrake);
    this.angleMotor.setIdleMode(IdleMode.kBrake);

    this.angleMotor.setInverted(false);

    this.angleMotor.getEncoder().setPosition(0);
  }

  public void setCollectSpeed(double speed) {
    this.collectMotor.set(speed);
  }

  public void setGraspRotations(double rotations) {
    this.angleMotor.getPIDController().setReference(rotations, ControlType.kPosition);
  }

  public void disableGrasp() {
    this.angleMotor.set(0);
  }

  public double getGraspRotations() {
    return this.angleMotor.getEncoder().getPosition();
  }

  public double getCurrentGrasp() {
    return this.angleMotor.getOutputCurrent();
  }

  public double getCurrentCollect() {
    return this.collectMotor.getOutputCurrent();
  }

  public void setGraspPower(double power) {
    this.angleMotor.set(power);
  }

  public boolean collectAtCurrentLimit() {
    return this.collectMotor.getOutputCurrent() >= collectCurrentLimit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Grasp Rotations", getGraspRotations());
  }
}
