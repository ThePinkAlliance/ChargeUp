// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private double powerLimit;

  /** Creates a new Mnipulator. */
  public ManipulatorSubsystem(int leftMotorId, int rightMotorId) {
    this.leftMotor = new CANSparkMax(leftMotorId, MotorType.kBrushless);
    this.rightMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);

    this.leftMotor.setSmartCurrentLimit(35);
    this.rightMotor.setSmartCurrentLimit(35);

    this.leftMotor.setInverted(false);
    this.rightMotor.setInverted(true);

    this.leftMotor.getPIDController().setP(.1);
    this.rightMotor.getPIDController().setP(.1);

    this.powerLimit = 0.4;
  }

  public void setPositionTargetRight(double current) {
    this.rightMotor.getPIDController().setReference(current, ControlType.kPosition);
  }

  public void setPositionTargetLeft(double current) {
    this.leftMotor.getPIDController().setReference(current, ControlType.kPosition);
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
  }
}
