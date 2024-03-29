// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    this.turretController.setOpenLoopRampRate(0.3);
    this.turretController.setClosedLoopRampRate(0.0);
    this.turretController.setInverted(false);
    this.useFaker = RobotBase.isSimulation();

    this.neoFaker = new Faker();

    // setEncoderPositions(91.43);
    setEncoderPositions(0);

    this.powerLimit = 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (useFaker) {
      neoFaker.update();
    }

    SmartDashboard.putNumber("Turret Rotations", turretController.getEncoder().getPosition());
  }

  public CANSparkMax getCanSparkMax() {
    return turretController;
  }

  public void setEncoderPositions(double pos) {
    this.turretController.getEncoder().setPosition(pos);
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
      rotations = neoFaker.getPosition() / 366;
    } else {
      rotations = this.turretController.getEncoder().getPosition() / Constants.TurretConstants.FULL_MOTOR_ROTATIONS;
    }

    return rotations * 360;
  }

  public double getTurretPosition() {
    return this.turretController.getEncoder().getPosition();
  }

  public boolean isMoving() {
    return this.turretController.get() >= 0.1;
  }
}

class Faker {
  private final double rotPerSec;

  private double currentPosition = 0;
  private double currentPower = 0;
  private double powerLimit = 1;

  public Faker() {
    this(366);
  }

  public Faker(double full_rot_ticks) {
    this.rotPerSec = full_rot_ticks / 60;
  }

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