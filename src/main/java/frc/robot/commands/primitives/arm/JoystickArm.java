// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitives.arm;

import java.util.List;
import java.util.function.Supplier;

import com.ThePinkAlliance.core.math.LinearInterpolationTable;
import com.ThePinkAlliance.core.math.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class JoystickArm extends CommandBase {
  private ArmSubsystem armSubsystem;
  private Supplier<Double> extSupplier;
  private Supplier<Double> pivotSupplier;
  private LinearInterpolationTable feedforwardTable;

  /** Creates a new CommandExtend. */
  public JoystickArm(ArmSubsystem armSubsystem, Supplier<Double> extSupplier, Supplier<Double> pivotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.extSupplier = extSupplier;
    this.pivotSupplier = pivotSupplier;

    this.feedforwardTable = new LinearInterpolationTable(List.of(new Vector2d(80, 0.07), new Vector2d(86, 0.07),
        new Vector2d(98, 0.05), new Vector2d(107, 0.0787), new Vector2d(117, 0.0708), new Vector2d(122, 0.015),
        new Vector2d(133, 0.066), new Vector2d(140, 0.031), new Vector2d(148, 0.070), new Vector2d(157, 0.051),
        new Vector2d(167, 0.041), new Vector2d(178, 0.051), new Vector2d(190, 0.011)));

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = pivotSupplier.get();
    double pivotAngle = armSubsystem.getPivotAngle();
    double ff = feedforwardTable.interp(pivotAngle);

    if (Double.isNaN(ff) || Double.isInfinite(ff)) {
      ff = 0;
    }

    if ((Math.signum(input) == 1 || Math.signum(input) == -1) && pivotAngle >= 84) {
      this.armSubsystem.commandPivot(input);
    } else if (Math.signum(input) == 1 && pivotAngle < 84) {
      this.armSubsystem.commandPivot(input);
    } else {
      this.armSubsystem.commandPivot(ff);
    }

    this.armSubsystem.commandExtend(extSupplier.get());

    SmartDashboard.putNumber("Pivot Demanded Power", armSubsystem.getPivotDemandedPower());
    SmartDashboard.putNumber("Pivot Power", input);
    SmartDashboard.putNumber("Pivot Angle", pivotAngle);

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
