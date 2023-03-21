// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveByGyroInfinity extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  PIDController thetaController;

  double distance;
  double startingTime;
  double desiredPitch;
  double startingHeading;
  double forwardSpeed;
  double speed;
  boolean doStop;

  /** Creates a new DriveStraightByGyro. */
  public DriveByGyroInfinity(double forwardSpeed, double desiredPitch, double speed, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.thetaController = new PIDController(.29, 0, 0);

    this.speed = speed;
    this.doStop = true;
    this.forwardSpeed = forwardSpeed;
    this.distance = distance;

    addRequirements(swerveSubsystem);
  }

  public DriveByGyroInfinity(double forwardSpeed, double desiredPitch, double speed, boolean doStop, SwerveSubsystem swerveSubsystem) {
    this(forwardSpeed, desiredPitch, speed, swerveSubsystem);

    this.doStop = doStop;
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

    this.thetaController.setSetpoint(swerveSubsystem.getHeading());
    this.startingHeading = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = swerveSubsystem.getHeading();
    double thetaEffort = MathUtil.clamp(thetaController.calculate(angle), -speed, speed);

    swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(new ChassisSpeeds(forwardSpeed, 0, thetaEffort)));
    SmartDashboard.putNumber("thetaEffort", thetaEffort);
    SmartDashboard.putNumber("Current Pitch", swerveSubsystem.getPitch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (doStop) {
      swerveSubsystem
      .setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = swerveSubsystem.getPitch() > desiredPitch;

    if (finished) {
      Telemetry.logData("---- DriveStraightByInfinity ----", "Finished Pitch: " + this.swerveSubsystem.getPitch(),
          DriveByGyroInfinity.class);
    }

    return finished;
  }
}
