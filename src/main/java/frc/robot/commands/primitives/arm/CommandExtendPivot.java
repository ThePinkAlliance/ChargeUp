// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitives.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class CommandExtendPivot extends CommandBase {
  private ArmSubsystem armSubsystem;
  private Supplier<Double> extSupplier;
  private Supplier<Double> pivotSupplier;

  /** Creates a new CommandExtend. */
  public CommandExtendPivot(ArmSubsystem armSubsystem, Supplier<Double> extSupplier, Supplier<Double> pivotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.extSupplier = extSupplier;
    this.pivotSupplier = pivotSupplier;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double extendPosition = armSubsystem.getExtensionRotations();
    double extPower = extSupplier.get();
    double pwrSign = Math.signum(extPower);

    if (pwrSign == 1) {
      this.armSubsystem.commandExtend(extPower);
    } else if (pwrSign == -1 && extendPosition >= 0.2) {
      this.armSubsystem.commandExtend(extPower);
    } else {
      this.armSubsystem.commandExtend(0);
    }

    SmartDashboard.putNumber("Extend Position", extendPosition);
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
