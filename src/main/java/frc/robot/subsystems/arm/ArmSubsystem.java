// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ThePinkAlliance.core.simulation.ctre.CtrePhysicsSim;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  TalonFX pivotMotor;
  CANCoder canCoder;
  Spark ledController;
  private double powerLimitPivot;
  private double pivotOffset;

  private double positionToHold = 0;

  public double getPositionToHold() {
    return positionToHold;
  }

  public void setPositionToHold(double positionToHold) {
    this.positionToHold = positionToHold;
  }

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(int pivotMotorId, int canCoderId, double pivotOffset, double powerLimitPivot) {

    this.ledController = new Spark(0);

    this.pivotMotor = new TalonFX(pivotMotorId);
    this.canCoder = new CANCoder(canCoderId);
    this.pivotOffset = pivotOffset;
    this.powerLimitPivot = powerLimitPivot;

    /* Factory default hardware to prevent unexpected behavior */
    pivotMotor.configFactoryDefault();
    pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.ArmConstants.kPIDLoopIdx,
        Constants.ArmConstants.kTimeoutMs);
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

  public void holdPosition() {
    pivotMotor.set(TalonFXControlMode.Position, getPositionToHold());
  }

  public void configureLED() {
    this.ledController.set(Constants.ArmConstants.LED_SPEED);
  }

  public void commandPivot(double input) {
    input = input * powerLimitPivot;

    this.pivotMotor.set(ControlMode.PercentOutput, input);
  }

  public void commandPivotUnsafe(double input) {
    this.pivotMotor.set(ControlMode.PercentOutput, input);
  }

  public double getPivotDemandedPower() {
    return this.pivotMotor.getMotorOutputPercent();
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

  public double getPivotVelocity() {
    return canCoder.getVelocity();
  }

  public void configureTalonFX_Position() {
    /* Factory default hardware to prevent unexpected behavior */
    // pivotMotor.configFactoryDefault();
    // configure close loop in slot1
    // pivotMotor.selectProfileSlot(Constants.ArmConstants.kSlotId_ForPosition,
    // Constants.ArmConstants.kPIDLoopIdx);
    // pivotMotor.config_kF(Constants.ArmConstants.kSlotId_ForPosition,
    // Constants.ArmConstants.POSITION_GAINS_FX.kF,
    // Constants.ArmConstants.kTimeoutMs);
    // pivotMotor.config_kP(Constants.ArmConstants.kSlotId_ForPosition,
    // Constants.ArmConstants.POSITION_GAINS_FX.kP,
    // Constants.ArmConstants.kTimeoutMs);
    // pivotMotor.config_kI(Constants.ArmConstants.kSlotId_ForPosition,
    // Constants.ArmConstants.POSITION_GAINS_FX.kI,
    // Constants.ArmConstants.kTimeoutMs);
    // pivotMotor.config_kD(Constants.ArmConstants.kSlotId_ForPosition,
    // Constants.ArmConstants.POSITION_GAINS_FX.kD,
    // Constants.ArmConstants.kTimeoutMs);
  }

  public void configureTalonFX_MotionMagic(double cruiseVelocity, double acceleration, int smoothingIntensity,
      double closedLoopThreshold) {
    /* Factory default hardware to prevent unexpected behavior */
    pivotMotor.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    /*
     * Note: we can connect the can coder and use it as the encoder for motion
     * magic.
     */
    pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.ArmConstants.kPIDLoopIdx,
        Constants.ArmConstants.kTimeoutMs);

    // /*
    // * set deadband to super small 0.001 (0.1 %).
    // * The default deadband is 0.04 (4 %)
    // */
    pivotMotor.configNeutralDeadband(Constants.ArmConstants.kPercentDeadband, Constants.ArmConstants.kTimeoutMs);

    // /**
    // * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
    // * have green LEDs when driving Talon Forward / Requesting Postiive Output
    // Phase
    // * sensor to have positive increment when driving Talon Forward (Green LED)
    // */
    // // pivotMotor.setSensorPhase(false);
    // /*
    // * Talon FX does not need sensor phase set for its integrated sensor
    // * This is because it will always be correct if the selected feedback device
    // is
    // * integrated sensor (default value)
    // * and the user calls getSelectedSensor* to get the sensor's
    // position/velocity.
    // *
    // * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
    // * sensor-phase
    // */
    // // pivotMotor.setSensorPhase(true);

    // /* Set relevant frame periods to be at least as fast as periodic rate */
    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
        Constants.ArmConstants.kTimeoutMs);
    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
        Constants.ArmConstants.kPeriodMs, Constants.ArmConstants.kTimeoutMs);

    // /* Set the peak and nominal outputs */
    pivotMotor.configNominalOutputForward(0, Constants.ArmConstants.kTimeoutMs);
    pivotMotor.configNominalOutputReverse(0, Constants.ArmConstants.kTimeoutMs);
    pivotMotor.configPeakOutputForward(Constants.ArmConstants.kNominalForwardPeak, Constants.ArmConstants.kTimeoutMs);
    pivotMotor.configPeakOutputReverse(Constants.ArmConstants.kNominalReversePeak, Constants.ArmConstants.kTimeoutMs);

    // /* Set Motion Magic gains in slot0 - see documentation */
    pivotMotor.selectProfileSlot(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.kPIDLoopIdx);
    pivotMotor.config_kF(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kF,
        Constants.ArmConstants.kTimeoutMs);
    pivotMotor.config_kP(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kP,
        Constants.ArmConstants.kTimeoutMs);
    pivotMotor.config_kI(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kI,
        Constants.ArmConstants.kTimeoutMs);
    pivotMotor.config_kD(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kD,
        Constants.ArmConstants.kTimeoutMs);

    // /* Set acceleration and vcruise velocity - see documentation */
    pivotMotor.configMotionCruiseVelocity(cruiseVelocity, Constants.ArmConstants.kTimeoutMs);
    pivotMotor.configMotionAcceleration(acceleration, Constants.ArmConstants.kTimeoutMs);
    pivotMotor.configMotionSCurveStrength(smoothingIntensity, Constants.ArmConstants.kTimeoutMs);
    pivotMotor.configAllowableClosedloopError(Constants.ArmConstants.kSlotIdx,
        Constants.ArmConstants.kAllowableCloseLoopError, Constants.ArmConstants.kTimeoutMs);
    pivotMotor.configAllowableClosedloopError(Constants.ArmConstants.kPIDLoopIdx, closedLoopThreshold,
        Constants.ArmConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch Motor Ticks", pivotMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Absolute Pitch", canCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Pitch", getArmPitch());
  }
}
