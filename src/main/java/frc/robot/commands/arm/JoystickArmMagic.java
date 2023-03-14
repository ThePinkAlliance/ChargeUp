// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.ArmSubsystem;

public class JoystickArmMagic extends CommandBase {
  private ArmSubsystem armSubsystem;
  private Supplier<Double> extSupplier;
  private Supplier<Double> pivotSupplier;
  private boolean updateHoldPosition;
  private final double PIVOT_DEADBAND = 0.05;
  private double ANGLE_FLOOR;
  private double ANGLE_CEILING;
  private double currentPosition;
  private TalonFX pivotMotor;

  private int kPIDLoopIdx = 0;
  private int kTimeoutMs = 10;
  private int kSlotIdx = 0;

  /** Creates a new CommandExtend. */
  public JoystickArmMagic(ArmSubsystem armSubsystem, Supplier<Double> extSupplier, Supplier<Double> pivotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.extSupplier = extSupplier;
    this.pivotSupplier = pivotSupplier;
    this.updateHoldPosition = false;
    this.ANGLE_FLOOR = 75;
    this.currentPosition = 0;
    this.ANGLE_CEILING = 218;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateHoldPosition = false;
    armSubsystem.getPivotTalon().configFactoryDefault();
    armSubsystem.getPivotTalon().config_kP(0, 0.1);
    this.pivotMotor = armSubsystem.getPivotTalon();
    this.armSubsystem.setPositionToHold(this.armSubsystem.getPivotTalon().getSelectedSensorPosition());
    this.currentPosition = this.armSubsystem.getPivotTalon().getSelectedSensorPosition();
    Telemetry.logData("Status", "Init", JoystickArmMagic.class);

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
    pivotMotor.config_kF(kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kF, kTimeoutMs);
    pivotMotor.config_kP(kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kP, kTimeoutMs);
    pivotMotor.config_kI(kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kI, kTimeoutMs);
    pivotMotor.config_kD(kSlotIdx, Constants.ArmConstants.MOTIONM_GAINS_FX.kD, kTimeoutMs);

    // /* Set acceleration and vcruise velocity - see documentation */
    pivotMotor.configMotionCruiseVelocity(Constants.ArmConstants.MAX_CRUISE_VELOCITY, kTimeoutMs);
    pivotMotor.configMotionAcceleration(Constants.ArmConstants.MAX_ACCELERATION, kTimeoutMs);
    pivotMotor.configMotionSCurveStrength(2, kTimeoutMs);

    pivotMotor.setSelectedSensorPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = Math.abs(pivotSupplier.get()) > PIVOT_DEADBAND ? pivotSupplier.get() : 0;
    double pivotAngle = armSubsystem.getArmPitch();
    double realPositionTicks = Constants.convertDegreesToPitchTicks(pivotAngle);

    // 512 is the tick multiplier.
    double ticksToWantedPosition = input * 512;

    // if ((this.armSubsystem.getArmPitch() < ANGLE_FLOOR && Math.signum(input) == 1
    // || this.armSubsystem.getArmPitch() > ANGLE_FLOOR && Math.signum(input) == -1)
    // || (this.armSubsystem.getArmPitch() > ANGLE_CEILING && Math.signum(input) ==
    // -1
    // || this.armSubsystem.getArmPitch() < ANGLE_CEILING && Math.signum(input) ==
    // 1)) {
    currentPosition = ticksToWantedPosition + currentPosition;

    this.armSubsystem.getPivotTalon().set(ControlMode.MotionMagic, currentPosition);
    // } else {
    // armSubsystem.commandPivot(0);
    // }

    double val = extSupplier.get();
    // Cube law on extended input
    // val = val * Math.abs(val);
    val = val * val * val;
    this.armSubsystem.commandExtend(val * -1);

    SmartDashboard.putNumber("Pivot Demanded Power", armSubsystem.getPivotDemandedPower());
    SmartDashboard.putNumber("Pivot Power", input);
    SmartDashboard.putNumber("Pivot Angle", pivotAngle);

    SmartDashboard.putNumber("Extend Current", armSubsystem.getExtendCurrent());
    SmartDashboard.putNumber("Extend Position", armSubsystem.getExtendedPosition());
    SmartDashboard.putNumber("Extend Distance", armSubsystem.getExtensionDistance());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.commandPivot(0);
    Telemetry.logData("Status", "Ended", JoystickArmMagic.class);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
