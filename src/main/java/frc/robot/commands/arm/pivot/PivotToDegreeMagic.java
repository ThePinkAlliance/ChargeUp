// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import java.util.List;
import java.util.function.Supplier;

import com.ThePinkAlliance.core.math.LinearInterpolationTable;
import com.ThePinkAlliance.core.math.Vector2d;
import com.ThePinkAlliance.core.util.GainsFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class PivotToDegreeMagic extends CommandBase {
  private final double angleFactor;

  private int smoothingIntensity;
  private double cruiseVelocity;
  private double acceleration;
  private double desiredAngle;
  private ArmSubsystem armSubsystem;
  private GainsFX gains;
  private TalonFX pivotMotor;
  private int kSlotIdx;
  private int kTimeoutMs;
  private int kPIDLoopIdx;
  private LinearInterpolationTable feedforwardTable;
  private Supplier<Boolean> safeToContinue;

  private double initialAngle;
  private boolean motorCommanded;

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreeMagic(double desiredAngle, GainsFX gains, Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desiredAngle = desiredAngle;
    this.safeToContinue = safeToContinue;
    this.armSubsystem = armSubsystem;
    this.gains = gains;
    this.pivotMotor = armSubsystem.getPivotTalon();
    this.feedforwardTable = new LinearInterpolationTable(List.of(new Vector2d(71, 0.0787), new Vector2d(74, 0.055),
        new Vector2d(77.78, 0.082), new Vector2d(94.30, 0.078),
        new Vector2d(122, 0.074), new Vector2d(130, 0.070), new Vector2d(145, 0.062), new Vector2d(180, 0)));

    this.kSlotIdx = 0;
    this.kTimeoutMs = 10;
    this.kPIDLoopIdx = 0;
    this.angleFactor = 36.3 / 57306;
    this.smoothingIntensity = 0;
    this.acceleration = 2000;
    this.cruiseVelocity = 2040;

    this.motorCommanded = false;
    this.initialAngle = 0;

    addRequirements(armSubsystem);
  }

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreeMagic(double desiredAngle, double cruiseVelocity, double acceleration, GainsFX gains,
      Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
    this(desiredAngle, gains, safeToContinue, armSubsystem);

    this.acceleration = acceleration;
    this.cruiseVelocity = cruiseVelocity;
  }

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreeMagic(double desiredAngle, double cruiseVelocity, double acceleration, int smoothingIntensity,
      GainsFX gains,
      Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
    this(desiredAngle, gains, safeToContinue, armSubsystem);

    this.acceleration = acceleration;
    this.cruiseVelocity = cruiseVelocity;
    this.smoothingIntensity = smoothingIntensity;
  }

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreeMagic(double desiredAngle, int smoothingIntensity, GainsFX gains,
      Supplier<Boolean> safeToContinue, ArmSubsystem armSubsystem) {
    this(desiredAngle, gains, safeToContinue, armSubsystem);

    this.smoothingIntensity = smoothingIntensity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.motorCommanded = false;

    /* Factory default hardware to prevent unexpected behavior */
    pivotMotor.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    /*
     * Note: we can connect the can coder and use it as the encoder for motion
     * magic.
     */
    pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx,
        kTimeoutMs);

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
    pivotMotor.configNeutralDeadband(0.001, kTimeoutMs);

    /**
     * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    pivotMotor.setSensorPhase(false);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // pivotMotor.setSensorPhase(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

    /* Set the peak and nominal outputs */
    pivotMotor.configNominalOutputForward(0, kTimeoutMs);
    pivotMotor.configNominalOutputReverse(0, kTimeoutMs);
    pivotMotor.configPeakOutputForward(1, kTimeoutMs);
    pivotMotor.configPeakOutputReverse(-1, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    pivotMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    pivotMotor.config_kF(kSlotIdx, gains.kF, kTimeoutMs);
    pivotMotor.config_kP(kSlotIdx, gains.kP, kTimeoutMs);
    pivotMotor.config_kI(kSlotIdx, gains.kI, kTimeoutMs);
    pivotMotor.config_kD(kSlotIdx, gains.kD, kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    pivotMotor.configMotionCruiseVelocity(cruiseVelocity, kTimeoutMs);
    pivotMotor.configMotionAcceleration(acceleration, kTimeoutMs);
    pivotMotor.configMotionSCurveStrength(smoothingIntensity);

    initialAngle = armSubsystem.getArmPitch();

    pivotMotor.setSelectedSensorPosition(0);

    System.out.println("---- Pivot Init ----");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = pivotMotor.getSelectedSensorPosition();
    double currentPitch = ((currentPosition + .001) / 1578.6776859504) + initialAngle;
    boolean isSafe = safeToContinue.get();

    if (!motorCommanded) {
      /* Convert the desired angle in degrees into the required position in ticks. */
      double desiredPosition = (desiredAngle - initialAngle) / angleFactor;

      pivotMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);

      SmartDashboard.putNumber("desiredPos", desiredPosition);

      this.motorCommanded = true;
    }
    // else if (motorCommanded && !isSafe) {
    // double ff = feedforwardTable.interp(armSubsystem.getArmPitch());

    // if (Double.isNaN(ff) || Double.isInfinite(ff)) {
    // ff = 0;
    // }

    // SmartDashboard.putNumber("ff", ff);

    // pivotMotor.set(ControlMode.PercentOutput, ff);
    // this.motorCommanded = false;
    // }

    SmartDashboard.putBoolean("isSafe", isSafe);
    SmartDashboard.putBoolean("isCommanded", motorCommanded);
    SmartDashboard.putNumber("currentAngle", currentPitch);
    SmartDashboard.putNumber("desiredAngle", desiredAngle);
    SmartDashboard.putNumber("InitalAngle", initialAngle);
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

    SmartDashboard.putNumber("mDiff", diff);
    SmartDashboard.putNumber("mCurrentPosition", currentPosition);
    SmartDashboard.putNumber("mDesiredPosition", desiredPosition);

    return (diff <= 200);
  }
}
