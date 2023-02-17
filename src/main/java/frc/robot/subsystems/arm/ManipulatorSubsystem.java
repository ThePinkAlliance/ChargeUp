// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
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

    this.leftMotor.setSmartCurrentLimit(20);
    this.rightMotor.setSmartCurrentLimit(20);

    this.powerLimit = 0.2;
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
