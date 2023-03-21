// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraightByGyro extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  TrapezoidProfile.Constraints constraints;
  PIDController thetaController;
  PIDController xController;

  double distance;
  double startingTime;
  double startingHeading;

  private final double MAX_SPEED;

  /** Creates a new DriveStraightByGyro. */
  public DriveStraightByGyro(double distance, TrapezoidProfile.Constraints constraints,
      SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.constraints = constraints;
    this.thetaController = new PIDController(.29, 0, 0);
    this.xController = new PIDController(4, 0, 0.025);

    this.MAX_SPEED = 3;
    this.distance = distance;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double xLocation = swerveSubsystem.getPose().getX();
    double xLocationTarget = distance + xLocation;

    SmartDashboard.putNumber("Starting Position", xLocation);
    SmartDashboard.putNumber("Target Position", xLocationTarget);
    SmartDashboard.putNumber("Starting Angle", swerveSubsystem.getHeading());

    this.startingTime = Timer.getFPGATimestamp();

    this.xController.setSetpoint(xLocationTarget);
    this.thetaController.setSetpoint(swerveSubsystem.getHeading());
    this.startingHeading = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xLocation = swerveSubsystem.getPose().getX();
    double angle = swerveSubsystem.getHeading();
    double thetaEffort = MathUtil.clamp(thetaController.calculate(angle), -MAX_SPEED, MAX_SPEED);
    double xEffort = MathUtil.clamp(xController.calculate(xLocation), -MAX_SPEED, MAX_SPEED);

    swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(new ChassisSpeeds(xEffort, 0, thetaEffort)));

    SmartDashboard.putNumber("thetaEffort", thetaEffort);
    SmartDashboard.putNumber("xLocation", swerveSubsystem.getPose().getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem
        .setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = this.xController.atSetpoint();

    if (finished) {
      Telemetry.logData("---- DriveStraightByGyro ----", "Finished At: " + this.xController.getPositionError(),
          DriveStraightByGyro.class);
    }

    return finished;
  }
}
