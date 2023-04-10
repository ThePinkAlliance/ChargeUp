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

public class DriveStraightByGyroStrafeLocked extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  PIDController thetaController;
  PIDController xController;
  PIDController yController;

  double distance;
  double startingTime;
  double startingHeading;
  double speed;
  boolean doStop;

  /** Creates a new DriveStraightByGyro. */
  public DriveStraightByGyroStrafeLocked(double distance, double speed, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.thetaController = new PIDController(.5, 0.05, 0);
    this.xController = new PIDController(4, 0, 0.025);
    this.yController = new PIDController(4, 0, 0.025);

    /*
     * The controller needs an I gain later.
     */
    this.xController.setTolerance(0.155);
    this.yController.setTolerance(0.155);

    /*
     * Continuous input was not enabled during our tests at slf this could be one
     * cause.
     */
    this.thetaController.enableContinuousInput(-180, 180);

    this.speed = speed;
    this.doStop = true;
    this.distance = distance;

    SmartDashboard.putNumber("Current Heading", 0);

    addRequirements(swerveSubsystem);
  }

  public DriveStraightByGyroStrafeLocked(double distance, double speed, boolean doStop,
      SwerveSubsystem swerveSubsystem) {
    this(distance, speed, swerveSubsystem);

    this.doStop = doStop;
  }

  public DriveStraightByGyroStrafeLocked configureTolerence(double tol) {
    this.xController.setTolerance(tol);

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double xLocation = swerveSubsystem.getEstimatedPose().getX();
    double yLocation = swerveSubsystem.getEstimatedPose().getY();
    double xLocationTarget = distance + xLocation;

    SmartDashboard.putNumber("Starting Position", xLocation);
    SmartDashboard.putNumber("Target Position", xLocationTarget);
    SmartDashboard.putNumber("Starting Angle", swerveSubsystem.getHeading());
    SmartDashboard.putNumber("Starting Heading", swerveSubsystem.getHeading());

    this.startingTime = Timer.getFPGATimestamp();

    this.xController.setSetpoint(xLocationTarget);
    this.yController.setSetpoint(yLocation);
    this.thetaController.setSetpoint(swerveSubsystem.getYaw());
    this.startingHeading = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xLocation = swerveSubsystem.getEstimatedPose().getX();
    double yLocation = swerveSubsystem.getEstimatedPose().getY();
    double angle = swerveSubsystem.getYaw();
    double calculatedTheta = thetaController.calculate(angle);
    double calculatedX = xController.calculate(xLocation);
    double calculatedY = xController.calculate(yLocation);
    double thetaEffort = MathUtil.clamp(calculatedTheta, -0.5, 0.5);
    double xEffort = MathUtil.clamp(calculatedX, -speed, speed);
    double yEffort = MathUtil.clamp(calculatedY, -speed, speed);

    swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(new ChassisSpeeds(xEffort, yEffort, thetaEffort)));

    SmartDashboard.putNumber("thetaEffort", thetaEffort);
    SmartDashboard.putNumber("Theta Diff", thetaController.getPositionError());
    SmartDashboard.putNumber("xEffort", xEffort);
    SmartDashboard.putNumber("Calculated Theta", calculatedTheta);
    SmartDashboard.putNumber("Calculated X", calculatedX);
    SmartDashboard.putNumber("Current Heading", angle);
    SmartDashboard.putNumber("Location Diff", xController.getPositionError());
    SmartDashboard.putNumber("xLocation", swerveSubsystem.getEstimatedPose().getX());
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
    boolean finished = this.xController.atSetpoint();

    if (finished) {
      Telemetry.logData("---- DriveStraightByGyro ----", "Finished At: " + this.xController.getPositionError(),
          DriveStraightByGyroStrafeLocked.class);
    }

    return finished;
  }
}
