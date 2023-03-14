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

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.ArmSubsystem;

public class PivotToDegreePosition extends CommandBase {
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
  private Supplier<Boolean> safeToContinue;

  private double initialAngle;
  private boolean isCommanded;

  private boolean isFinished;
  private Watchdog watchdog;
  private final double WATCHDOG_TIMEOUT = 4.5;
  private boolean didConfigure;

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreePosition(double desiredAngle, GainsFX gains, Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desiredAngle = desiredAngle;
    this.safeToContinue = safeToContinue;
    this.armSubsystem = armSubsystem;
    this.gains = gains;
    this.didConfigure = false;
    this.pivotMotor = armSubsystem.getPivotTalon();
    this.isCommanded = false;

    this.kSlotIdx = 0;
    this.kTimeoutMs = 10;
    this.kPIDLoopIdx = 0;
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
  public PivotToDegreePosition(double desiredAngle, double cruiseVelocity, double acceleration, GainsFX gains,
      Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
    this(desiredAngle, gains, safeToContinue, armSubsystem);

    this.acceleration = acceleration;
    this.cruiseVelocity = cruiseVelocity;
  }

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreePosition(double desiredAngle, double cruiseVelocity, double acceleration, int smoothingIntensity,
      GainsFX gains,
      Supplier<Boolean> safeToContinue,
      ArmSubsystem armSubsystem) {
    this(desiredAngle, gains, safeToContinue, armSubsystem);

    this.acceleration = acceleration;
    this.cruiseVelocity = cruiseVelocity;
    this.smoothingIntensity = smoothingIntensity;
  }

  /** Creates a new PivotToDegreeMagic. */
  public PivotToDegreePosition(double desiredAngle, int smoothingIntensity, GainsFX gains,
      Supplier<Boolean> safeToContinue, ArmSubsystem armSubsystem) {
    this(desiredAngle, gains, safeToContinue, armSubsystem);

    this.smoothingIntensity = smoothingIntensity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
    this.isCommanded = false;

    /* Factory default hardware to prevent unexpected behavior */
    pivotMotor.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    /*
     * Note: we can connect the can coder and use it as the encoder for motion
     * magic.
     */
    pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        kPIDLoopIdx,
        kTimeoutMs);

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
    pivotMotor.configNeutralDeadband(0.001, kTimeoutMs);

    // /* Set relevant frame periods to be at least as fast as periodic rate */
    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
        kTimeoutMs);
    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
        10, kTimeoutMs);

    // /* Set the peak and nominal outputs */
    pivotMotor.configNominalOutputForward(0, kTimeoutMs);
    pivotMotor.configNominalOutputReverse(0, kTimeoutMs);
    pivotMotor.configPeakOutputForward(1, kTimeoutMs);
    pivotMotor.configPeakOutputReverse(-1, kTimeoutMs);

    // /* Set Motion Magic gains in slot0 - see documentation */
    pivotMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    pivotMotor.config_kF(kSlotIdx, gains.kF, kTimeoutMs);
    pivotMotor.config_kP(kSlotIdx, gains.kP, kTimeoutMs);
    pivotMotor.config_kI(kSlotIdx, gains.kI, kTimeoutMs);
    pivotMotor.config_kD(kSlotIdx, gains.kD, kTimeoutMs);

    // /* Set acceleration and vcruise velocity - see documentation */
    pivotMotor.configMotionCruiseVelocity(cruiseVelocity, kTimeoutMs);
    pivotMotor.configMotionAcceleration(acceleration, kTimeoutMs);
    pivotMotor.configMotionSCurveStrength(smoothingIntensity, kTimeoutMs);

    initialAngle = armSubsystem.getArmPitch();

    pivotMotor.setSelectedSensorPosition(0);

    watchdog.reset();
    watchdog.enable();
    System.out.println("---- Pivot Init ----");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = pivotMotor.getSelectedSensorPosition();
    double currentPitch = ((currentPosition + .001) / 1578.6776859504) + initialAngle;
    double desiredPosition = (desiredAngle - initialAngle) / angleFactor;

    boolean isSafe = safeToContinue.get();

    if (!isCommanded) {
      pivotMotor.set(TalonFXControlMode.Position, desiredPosition);

      this.isCommanded = true;
    }

    Telemetry.logData("isSafe", isSafe, PivotToDegreePosition.class);
    Telemetry.logData("isFinished", isFinished, PivotToDegreePosition.class);
    Telemetry.logData("currentAngle", currentPitch, PivotToDegreePosition.class);
    Telemetry.logData("desiredPosition", desiredPosition, PivotToDegreePosition.class);
    Telemetry.logData("desiredAngle", desiredAngle, PivotToDegreePosition.class);
    Telemetry.logData("initalAngle", initialAngle, PivotToDegreePosition.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.setPositionToHold(pivotMotor.getSelectedSensorPosition());
    System.out.println("---- Pivot Command Terminated ----");

    // pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition = pivotMotor.getSelectedSensorPosition();
    double desiredPosition = (desiredAngle - initialAngle) / angleFactor;
    double diff = Math.abs(desiredPosition - currentPosition);
    boolean canExit = diff <= 130;

    Telemetry.logData("Pitch Difference", diff, PivotToDegreePosition.class);
    Telemetry.logData("Current Pitch Position", currentPosition, PivotToDegreePosition.class);
    Telemetry.logData("canExit (diff under threshold)", canExit, PivotToDegreePosition.class);
    Telemetry.logData("watchDog expired", watchdog.isExpired(), PivotToDegreePosition.class);

    return (canExit || watchdog.isExpired());
  }
}
