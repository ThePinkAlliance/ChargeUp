// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitives.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class JoystickExtend extends CommandBase {
  private ArmSubsystem armSubsystem;
  private Supplier<Double> extSupplier;
  private Supplier<Double> pivotSupplier;

  private ArmFeedforward feedforward;

  /** Creates a new CommandExtend. */
  public JoystickExtend(ArmSubsystem armSubsystem, Supplier<Double> extSupplier, Supplier<Double> pivotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.extSupplier = extSupplier;
    this.pivotSupplier = pivotSupplier;
    this.feedforward = new ArmFeedforward(0, 0.1, 0);

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("kS", 0);
    SmartDashboard.putNumber("kG", 0);
    SmartDashboard.putNumber("kV", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double extendPosition = armSubsystem.getExtensionRotations();
    double kS = SmartDashboard.getNumber("kS", 0);
    double kG = SmartDashboard.getNumber("kG", 0);
    double kV = SmartDashboard.getNumber("kV", 0);

    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    double ff = feedforward.calculate(armSubsystem.getPivotAngle() / (180 / Math.PI),
        armSubsystem.getPivotVelocity() / (180 / Math.PI));

    double input = pivotSupplier.get();

    if ((Math.signum(input) == 1 || Math.signum(input) == -1) && armSubsystem.getPivotAngle() >= 79) {
      this.armSubsystem.commandPivot(input);
    } else if (Math.signum(input) == 1 && armSubsystem.getPivotAngle() < 79) {
      this.armSubsystem.commandPivot(input);
    } else {
      this.armSubsystem.commandPivot(0);
    }

    this.armSubsystem.commandExtend(extSupplier.get());

    SmartDashboard.putNumber("xFF", ff);
    SmartDashboard.putNumber("Pivot Voltage", armSubsystem.getPivotVoltage());
    SmartDashboard.putNumber("Extend Position", extendPosition);
    SmartDashboard.putNumber("Extend Current", armSubsystem.getExtendCurrent());
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
