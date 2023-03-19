// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ThePinkAlliance.core.simulation.ctre.CtrePhysicsSim;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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

public class ArmSubsystem extends SubsystemBase {
  TalonFX pivotMotor;
  CANSparkMax extendMotor;
  RelativeEncoder extendEncoder;
  CANCoder canCoder;
  Spark ledController;

  private double powerLimitPivot;
  private double powerLimitExtend;
  private double maxRotations = 71;
  private double maxDistanceMeters = 0;
  private double pivotOffset;

  private double desiredExtendRotations = 0;
 
  private double positionToHold = 0;

  public double getPositionToHold() {
    return positionToHold;
  }

  public void setPositionToHold(double positionToHold) {
    this.positionToHold = positionToHold;
  }

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(int pivotMotorId, int extendMotorId, int canCoderId, double pivotOffset, double powerLimitPivot,
      double powerLimitExtend /*,
      Constraints constraints */) {
    
    this.ledController = new Spark(0);

    this.pivotMotor = new TalonFX(pivotMotorId);
    this.extendMotor = new CANSparkMax(extendMotorId, MotorType.kBrushless);
    this.canCoder = new CANCoder(canCoderId);
    
    this.extendEncoder = extendMotor.getEncoder();
    this.pivotOffset = pivotOffset;
    this.powerLimitPivot = powerLimitPivot;
    this.powerLimitExtend = powerLimitExtend;

    this.extendMotor.getPIDController().setP(0.1);
    this.extendMotor.getPIDController().setOutputRange(-0.65, 0.65);
    this.extendMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ArmConstants.EXTENDER_90_MAX_LIMIT);
    this.extendMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ArmConstants.EXTENDER_MIN_LIMIT);
    this.extendMotor.setOpenLoopRampRate(.5);
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    this.extendMotor.getEncoder().setPosition(0);
    this.extendMotor.setInverted(false);
    this.extendMotor.setIdleMode(IdleMode.kBrake);

    this.pivotMotor.configOpenloopRamp(0.5);
    this.pivotMotor.setNeutralMode(NeutralMode.Brake);
    this.pivotMotor.setInverted(true);
    this.pivotMotor.setSelectedSensorPosition(0);

    SmartDashboard.putNumber("pivot-kP", 0);
    SmartDashboard.putNumber("pivot-kI", 0);
    SmartDashboard.putNumber("pivot-kD", 0);

    if (RobotBase.isSimulation()) {
      CtrePhysicsSim.getInstance().addTalonFX(pivotMotor, 0.5, 5100);
    }
  }

  public TalonFX getPivotTalon() {
    return this.pivotMotor;
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

  public void configureLED() {
    this.ledController.set(Constants.ArmConstants.LED_SPEED);
  }

  public boolean atExtensionSetpoint() {
    return Math.abs(desiredExtendRotations - this.extendMotor.getEncoder().getPosition()) < Constants.ArmConstants.EXTENDER_MARGIN_OF_ERROR;
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

  public void commandPivot(double input) {
    input = input * powerLimitPivot;

    this.pivotMotor.set(ControlMode.PercentOutput, input);
  }

  public void commandExtendUnsafe(double input) {
    this.extendMotor.set(input);
  }

  public void commandPivotUnsafe(double input) {
    this.pivotMotor.set(ControlMode.PercentOutput, input);
  }

  public double getPivotDemandedPower() {
    return this.pivotMotor.getMotorOutputPercent();
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

  public double getPivotVoltage() {
    return pivotMotor.getMotorOutputVoltage();
  }

  public double getArmPitch() {
    /**
     * Name Raw Degrees, Command Degrees, Valid?
     * Full Down 134.561, 72.16,
     */
    return (canCoder.getAbsolutePosition() - Constants.ArmConstants.PITCH_FLOOR_ABSOLUTE) + pivotOffset;
  }

  @Deprecated
  public double getPivotAngle() {
    return getArmPitch();
  }

  public double getPivotVelocity() {
    return canCoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("[ARM/PERIODIC] Extend Position: " +
    // extendMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("[EXTEND] Ticks", getExtensionRotations());
    SmartDashboard.putNumber("Pitch Motor Ticks", pivotMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Absolute Pitch", canCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Pivot Pitch", this.getPivotAngle());
  }
}
