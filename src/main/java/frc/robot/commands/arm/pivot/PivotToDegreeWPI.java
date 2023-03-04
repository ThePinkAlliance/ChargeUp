// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import java.util.List;

import com.ThePinkAlliance.core.math.LinearInterpolationTable;
import com.ThePinkAlliance.core.math.Vector2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

@Deprecated
public class PivotToDegreeWPI extends CommandBase {
  ArmSubsystem armSubsystem;
  boolean isFinished;
  LinearInterpolationTable feedforwardTable;
  double desiredAngle;
  PIDController controller;
  double startingTime;
  double epochTime;
  TrapezoidProfile profile;
  TrapezoidProfile.State goal;
  TrapezoidProfile.State lastRef;

  /** Creates a new PivotToDegree. */
  public PivotToDegreeWPI(ArmSubsystem armSubsystem, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
    this.isFinished = false;
    this.controller = new PIDController(0.5, 0.11, 0);

    this.feedforwardTable = new LinearInterpolationTable(List.of(new Vector2d(71, 0.0787), new Vector2d(74, 0.055),
        new Vector2d(77.78, 0.082), new Vector2d(94.30, 0.078),
        new Vector2d(122, 0.074), new Vector2d(130, 0.070), new Vector2d(145, 0.062), new Vector2d(180, 0)));

    SmartDashboard.putNumber("pivot-kP", controller.getP());
    SmartDashboard.putNumber("pivot-kI", controller.getI());
    SmartDashboard.putNumber("pivot-kD", controller.getD());

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    double targetRad = desiredAngle * (Math.PI / 180);

    double kP = SmartDashboard.getNumber("pivot-kP", controller.getP());
    double kI = SmartDashboard.getNumber("pivot-kI", controller.getI());
    double kD = SmartDashboard.getNumber("pivot-kD", controller.getD());

    SmartDashboard.putNumber("Pivot Target", desiredAngle * (Math.PI / 180));

    goal = new TrapezoidProfile.State(targetRad, 0);
    lastRef = new TrapezoidProfile.State();
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 1), goal,
        new TrapezoidProfile.State(armSubsystem.getPivotAngle() * Math.PI / 180, 0));

    controller.setPID(kP, kI, kD);

    startingTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tDelta = startingTime - Timer.getFPGATimestamp();
    double epochDelta = Timer.getFPGATimestamp() - epochTime;
    double currentAngle = armSubsystem.getPivotAngle() * (Math.PI / 180);
    double ff = feedforwardTable.interp(currentAngle);

    if (Double.isNaN(ff) || Double.isInfinite(ff)) {
      ff = 0;
    }

    // double vel = (lastRef.position - currentAngle) / epochDelta;
    TrapezoidProfile.State curState = new TrapezoidProfile.State(currentAngle, 0);
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 1), goal,
        curState);
    TrapezoidProfile.State state = profile.calculate(tDelta);
    double controlEffort = (goal.position - state.position) / state.position;

    SmartDashboard.putNumber("Pivot Position", currentAngle);
    SmartDashboard.putNumber("Pivot Control Effort", controlEffort);
    SmartDashboard.putNumber("epochDelta", epochDelta);
    SmartDashboard.putNumber("State Position", state.position);
    SmartDashboard.putNumber("State Velocity", state.velocity);

    if (this.profile.isFinished(tDelta)) {
      isFinished = true;
    }

    this.armSubsystem.commandPivot(controlEffort + ff);

    epochTime = Timer.getFPGATimestamp();
    lastRef = curState;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.commandPivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
