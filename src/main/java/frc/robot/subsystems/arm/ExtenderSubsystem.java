// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ThePinkAlliance.core.simulation.ctre.CtrePhysicsSim;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  CANSparkMax extendMotor;
  RelativeEncoder extendEncoder;
  private double powerLimitExtend;
  private double maxRotations = 71;
  private double maxDistanceMeters = 0;

  private double desiredExtendRotations = 0;
 
  private double positionToHold = 0;

  public double getPositionToHold() {
    return positionToHold;
  }

  public void setPositionToHold(double positionToHold) {
    this.positionToHold = positionToHold;
  }

  /** Creates a new ArmSubsystem. */
  public ExtenderSubsystem(int extendMotorId, double powerLimitExtend) {
    
   
    this.extendMotor = new CANSparkMax(extendMotorId, MotorType.kBrushless);
    this.extendEncoder = extendMotor.getEncoder();
    this.powerLimitExtend = powerLimitExtend;

    this.extendMotor.getPIDController().setP(0.1);
    this.extendMotor.getPIDController().setOutputRange(-0.65, 0.65);
    this.extendMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ExtenderConstants.EXTENDER_90_MAX_LIMIT);
    this.extendMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ExtenderConstants.EXTENDER_MIN_LIMIT);
    this.extendMotor.setOpenLoopRampRate(.5);
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    this.extendMotor.getEncoder().setPosition(0);
    this.extendMotor.setInverted(false);
    this.extendMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setExtenionRotations(double rotations) {
    // This will clip the commandable rotations between 0 and maxRotations.
    // if (rotations > maxRotations) {
    // rotations = maxRotations;
    // } else if (rotations < 0) {
    // rotations = 0;
    // }

    REVLibError err = extendMotor.getPIDController().setReference(
        rotations,
        ControlType.kPosition);

    if (err == REVLibError.kOk) {
      this.desiredExtendRotations = rotations;
    } else if (err == REVLibError.kSetpointOutOfRange) {
      System.err.println("[ARM, Extend]: Desired rotations out of range");
    } else {
      System.err.println("[ARM, Extend]: Error " + err.toString());
    }
  }

  public boolean atExtensionSetpoint() {
    return Math.abs(desiredExtendRotations - this.extendMotor.getEncoder().getPosition()) < Constants.ExtenderConstants.EXTENDER_MARGIN_OF_ERROR;
  }

  public double getExtendedPosition() {
    return this.extendMotor.getEncoder().getPosition();
  }

  public void enableExtendReverseSoftLimits() {
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void disableExtendReverseSoftLimits() {
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    extendMotor.getEncoder().setPosition(0);
  }

  public void enableExtendForwardSoftLimits() {
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  public void disableExtendForwardSoftLimits() {
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
  }

  public void commandExtend(double input) {
    input = input * powerLimitExtend;

    this.extendMotor.set(input);
  }

  public void commandExtendUnsafe(double input) {
    this.extendMotor.set(input);
  }

  public double getExtendCurrent() {
    return this.extendMotor.getOutputCurrent();
  }

  /**
   * Returns the extended distance in meters.
   */
  public double getExtensionDistance() {
    return extendEncoder.getPosition() * (maxDistanceMeters / maxRotations);
  }

  public double getExtensionRotations() {
    return extendMotor.getEncoder().getPosition();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("[ARM/PERIODIC] Extend Position: " +
    // extendMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("[EXTEND] Ticks", getExtensionRotations());
  }
}
