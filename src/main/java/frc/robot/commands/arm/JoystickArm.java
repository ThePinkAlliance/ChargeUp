// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

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

    this.feedforwardTable = new LinearInterpolationTable(List.of(new Vector2d(71, 0.0787), new Vector2d(74, 0.055),
        new Vector2d(77.78, 0.082), new Vector2d(94.30, 0.078),
        new Vector2d(122, 0.074), new Vector2d(130, 0.070), new Vector2d(145, 0.062), new Vector2d(180, 0)));

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

    // if ((Math.signum(input) == 1 || Math.signum(input) == -1) && pivotAngle >=
    // 84) {
    // this.armSubsystem.commandPivot(input);
    // } else if (Math.signum(input) == 1 && pivotAngle < 84) {
    // this.armSubsystem.commandPivot(input);
    // } else {
    // this.armSubsystem.commandPivot(ff);
    // }

    this.armSubsystem.commandPivot(input + ff);
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
