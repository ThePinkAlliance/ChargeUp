// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax turretController;
  private double powerLimit;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(int motorID) {
    this.turretController = new CANSparkMax(motorID, MotorType.kBrushless);
    this.turretController.setSmartCurrentLimit(20);

    this.powerLimit = 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void powerTurret(double input) {
    powerTurretUnsafe(input * powerLimit);
  }

  public void powerTurretUnsafe(double input) {
    this.turretController.set(input);
  }

  public double getTurretAngle() {
    double rotations = this.turretController.getEncoder().getPosition();
    double unRangedAngle = (rotations * (1 / 5)) * 360;

    return Math.IEEEremainder(unRangedAngle, 360);
  }
}
