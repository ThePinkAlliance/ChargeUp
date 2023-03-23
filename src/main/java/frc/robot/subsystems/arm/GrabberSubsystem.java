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
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax graspMotor;

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    this.graspMotor = new CANSparkMax(Constants.GrabberConstants.GRABBER_CAN_ID_GRASP, MotorType.kBrushless);
    this.intakeMotor = new CANSparkMax(Constants.GrabberConstants.GRABBER_CAN_ID_INTAKE, MotorType.kBrushless);

    this.graspMotor.setSmartCurrentLimit(Constants.GrabberConstants.GRABBER_GRASP_CURRENT_LIMIT);
    this.intakeMotor.setSmartCurrentLimit(Constants.GrabberConstants.GRABBER_INTAKE_CURRENT_LIMIT);

    this.graspMotor.getPIDController().setP(Constants.GrabberConstants.GRASP_GAINS_FX.kP);
    this.intakeMotor.getPIDController().setP(Constants.GrabberConstants.INTAKE_GAINS_FX.kP);

    this.intakeMotor.setIdleMode(IdleMode.kBrake);
    this.graspMotor.setIdleMode(IdleMode.kBrake);

    this.graspMotor.setInverted(false);
  }

  public void setIntakeSpeed(double speed) {
    this.intakeMotor.set(speed);
  }

  public void setGraspRotations(double rotations) {
    this.graspMotor.getPIDController().setReference(rotations, ControlType.kPosition);
  }

  public void disableGrasp() {
    this.graspMotor.set(0);
  }

  public double getGraspRotations() {
    return this.graspMotor.getEncoder().getPosition();
  }

  public double getGraspCurrent() {
    return this.graspMotor.getOutputCurrent();
  }

  public double getIntakeCurrent() {
    return this.intakeMotor.getOutputCurrent();
  }

  public void setGraspPower(double power) {
    this.graspMotor.set(power);
  }

  public boolean intakeAtCurrentLimit() {
    return getIntakeCurrent() >= Constants.GrabberConstants.GRABBER_INTAKE_CURRENT_LIMIT;
  }

  public boolean graspAtCustomCurrentLimit(double customLimit) {
    return getGraspCurrent() >= customLimit;
  }

  public void zeroGraspPosition() {
    this.graspMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Grasp Temp", graspMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Temp", intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Grasp Rotations", getGraspRotations());
  }
}
