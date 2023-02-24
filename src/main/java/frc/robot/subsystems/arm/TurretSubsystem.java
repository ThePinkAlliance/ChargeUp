// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax turretController;
  private double powerLimit;
  private Faker neoFaker;
  private boolean useFaker;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(int motorID) {
    this.turretController = new CANSparkMax(motorID, MotorType.kBrushless);
    this.turretController.setSmartCurrentLimit(20);
    this.turretController.setIdleMode(IdleMode.kBrake);
    this.useFaker = RobotBase.isSimulation();

    this.neoFaker = new Faker();

    this.powerLimit = 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (useFaker) {
      neoFaker.update();
    }
  }

  public void powerTurret(double input) {
    powerTurretUnsafe(input * powerLimit);
  }

  public void powerTurretUnsafe(double input) {
    if (useFaker) {
      neoFaker.setPower(input);
    } else {
      this.turretController.set(input);
    }
  }

  public double getTurretAngle() {
    double rotations = 0;

    if (useFaker) {
      rotations = neoFaker.getPosition();
    } else {
      rotations = this.turretController.getEncoder().getPosition();
    }

    return Math.IEEEremainder((rotations / 366) * 360, 360);
  }
}

class Faker {
  private double currentPosition = 0;
  private double currentPower = 0;
  private double rotPerSec = 366 / 60;
  private double powerLimit = 1;

  public void setPower(double p) {
    if (p > Math.copySign(powerLimit, 1)) {
      p = Math.copySign(powerLimit, 1);
    } else if (p < Math.copySign(powerLimit, -1)) {
      p = Math.copySign(powerLimit, -1);
    }

    this.currentPower = p;
  }

  public double getPosition() {
    return currentPosition;
  }

  public void update() {
    currentPosition += rotPerSec * currentPower;
  }
}