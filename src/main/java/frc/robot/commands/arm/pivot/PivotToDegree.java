// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import java.util.List;

import com.ThePinkAlliance.core.math.LinearInterpolationTable;
import com.ThePinkAlliance.core.math.Vector2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class PivotToDegree extends CommandBase {
  ArmSubsystem armSubsystem;
  boolean isFinished;
  LinearInterpolationTable feedforwardTable;
  double desiredAngle;
  PIDController controller;

  /** Creates a new PivotToDegree. */
  public PivotToDegree(ArmSubsystem armSubsystem, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
    this.isFinished = false;
    this.controller = new PIDController(0.5, 0, 0);

    this.feedforwardTable = new LinearInterpolationTable(List.of(new Vector2d(71, 0.0787), new Vector2d(74, 0.055),
        new Vector2d(77.78, 0.082), new Vector2d(94.30, 0.078),
        new Vector2d(122, 0.074), new Vector2d(130, 0.070), new Vector2d(145, 0.062), new Vector2d(180, 0)));

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    double kP = SmartDashboard.getNumber("pivot-kP", controller.getP());
    double kI = SmartDashboard.getNumber("pivot-kI", controller.getI());
    double kD = SmartDashboard.getNumber("pivot-kD", controller.getD());

    controller.setPID(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = armSubsystem.getPivotAngle();
    double ff = feedforwardTable.interp(currentAngle);
    double pidOutput = controller.calculate(currentAngle * (Math.PI / 180), desiredAngle * (Math.PI / 180));

    if (Double.isNaN(ff) || Double.isInfinite(ff)) {
      ff = 0;
    }

    SmartDashboard.putNumber("Pivot Position", currentAngle * (Math.PI / 180));
    SmartDashboard.putNumber("Pivot Control Effort", pidOutput);

    this.armSubsystem.commandPivot(pidOutput + ff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
