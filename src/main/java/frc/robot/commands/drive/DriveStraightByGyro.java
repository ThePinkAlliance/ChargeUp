// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  ProfiledPIDController xController;

  double distance;
  double speed;
  double startingTime;
  double startingHeading;

  /** Creates a new DriveStraightByGyro. */
  public DriveStraightByGyro(double distance, double speed, TrapezoidProfile.Constraints constraints,
      SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.constraints = constraints;
    this.thetaController = new PIDController(.1, 0, 0);
    this.xController = new ProfiledPIDController(3.5, 0, 0, new Constraints(1, 2));

    this.distance = distance;
    this.speed = speed;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double xLocation = swerveSubsystem.getPose().getX();
    double xLocationRelative = distance + xLocation;

    this.startingTime = Timer.getFPGATimestamp();

    this.xController.setGoal(new State(xLocationRelative, 0));
    this.thetaController.setSetpoint(swerveSubsystem.getHeading());
    this.startingHeading = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xLocation = swerveSubsystem.getPose().getX();
    double angle = swerveSubsystem.getHeading();
    double thetaEffort = thetaController.calculate(angle);
    double xEffort = xController.calculate(xLocation);

    swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(new ChassisSpeeds(xEffort, 0, thetaEffort)));

    SmartDashboard.putNumber("xEffort", xEffort);
    // SmartDashboard.putNumber("xEffort Processed", xEffort /
    // Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
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
