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

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private double powerLimit;
  private final int MAX_TEMP = 50;

  /** Creates a new Mnipulator. */
  public ManipulatorSubsystem(int leftMotorId, int rightMotorId) {
    this.leftMotor = new CANSparkMax(leftMotorId, MotorType.kBrushless);
    this.rightMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);

    this.leftMotor.setIdleMode(IdleMode.kBrake);
    this.rightMotor.setIdleMode(IdleMode.kBrake);

    this.leftMotor.setSmartCurrentLimit(32);
    this.rightMotor.setSmartCurrentLimit(32);

    this.leftMotor.setInverted(false);
    this.rightMotor.setInverted(true);

    this.leftMotor.getPIDController().setP(.2);
    this.rightMotor.getPIDController().setP(.2);

    this.powerLimit = 0.8;
  }

  public void setPositionTargetRight(double current) {
    setRightControl(current, ControlType.kPosition);
  }

  public void setPositionTargetLeft(double current) {
    setLeftControl(current, ControlType.kPosition);
  }

  public void setLeftControl(double input, ControlType type) {
    boolean isSafe = (leftMotor.getMotorTemperature() > MAX_TEMP) == false;

    if (isSafe) {
      this.leftMotor.getPIDController().setReference(input, type);
    } else {
      this.leftMotor.disable();
    }

    SmartDashboard.putBoolean("Left Temp Kill", isSafe);
  }

  public void setRightControl(double input, ControlType type) {
    boolean isSafe = (rightMotor.getMotorTemperature() > MAX_TEMP) == false;

    if (isSafe) {
      this.rightMotor.getPIDController().setReference(input, type);
    } else {
      this.rightMotor.disable();
    }

    SmartDashboard.putBoolean("Right Temp Kill", isSafe);
  }

  public void resetLeftEncoder() {
    this.leftMotor.getEncoder().setPosition(0);
  }

  public void resetRightEncoder() {
    this.rightMotor.getEncoder().setPosition(0);
  }

  public void setLeftPower(double input) {
    setLeftPowerUnsafe(input * powerLimit);
  }

  public void setRightPower(double input) {
    setRightPowerUnsafe(input * powerLimit);
  }

  public void setLeftPowerUnsafe(double input) {
    leftMotor.set(input);
  }

  public void setRightPowerUnsafe(double input) {
    rightMotor.set(input);
  }

  public double getRightCurrent() {
    return rightMotor.getOutputCurrent();
  }

  public double getLeftCurrent() {
    return leftMotor.getOutputCurrent();
  }

  public double getRightVelocity() {
    return rightMotor.getEncoder().getVelocity();
  }

  public double getLeftVelocity() {
    return leftMotor.getEncoder().getVelocity();
  }

  /**
   * Returns the current rotations of the motor.
   */
  public double getRightPosition() {
    return rightMotor.getEncoder().getPosition();
  }

  /**
   * Returns the current rotations of the motor.
   */
  public double getLeftPosition() {
    return leftMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Temp", this.leftMotor.getMotorTemperature());
    SmartDashboard.putNumber("Right Temp", this.rightMotor.getMotorTemperature());

    SmartDashboard.putNumber("Left Counts", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Counts", rightMotor.getEncoder().getPosition());
  }
}
