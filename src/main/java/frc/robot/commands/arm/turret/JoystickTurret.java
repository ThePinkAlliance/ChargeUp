// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

public class JoystickTurret extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private ArmSubsystem armSubsystem;
  private Supplier<Double> inputSupplier;
  private double minimumAngle;

  /** Creates a new CommandTurret. */
  public JoystickTurret(TurretSubsystem turretSubsystem, Supplier<Double> inputSupplier, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.armSubsystem = armSubsystem;
    this.inputSupplier = inputSupplier;
    this.minimumAngle = 90;

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = Math.abs(inputSupplier.get()) > 0.05 ? inputSupplier.get() * -1 : 0;

    if (armSubsystem.getArmPitch() >= minimumAngle && ((input > 0)
        || (input < 0 && turretSubsystem.getTurretAngle() > 0))) {
      this.turretSubsystem.powerTurret(input);
    } else {
      this.turretSubsystem.powerTurret(0);
    }

    SmartDashboard.putNumber("Turret Angle", turretSubsystem.getTurretAngle());
    SmartDashboard.putNumber("Turret Position", turretSubsystem.getTurretPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
