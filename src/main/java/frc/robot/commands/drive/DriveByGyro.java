// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveByGyro extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  PIDController thetaController;
  PIDController xController;
  PIDController yController;

  Translation2d targetLocation;
  double startingTime;
  double startingHeading;
  double speed;
  boolean doStop;

  /** Creates a new DriveStraightByGyro. */
  public DriveByGyro(Translation2d targetLocation, double speed, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.thetaController = new PIDController(.2, 0, 0);
    this.xController = new PIDController(4, 0, 0.025);
    this.yController = new PIDController(4.8, 0.005, 0.025);

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
    this.targetLocation = targetLocation;

    SmartDashboard.putNumber("Current Heading", 0);

    addRequirements(swerveSubsystem);
  }

  public DriveByGyro configureYTolerence(double tol) {
    this.yController.setTolerance(tol);

    return this;
  }

  public DriveByGyro configureXTolerence(double tol) {
    this.xController.setTolerance(tol);

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentLocation = swerveSubsystem.getEstimatedPose();
    Pose2d calculatedPosition = new Pose2d(currentLocation.getX() + targetLocation.getX(),
        currentLocation.getY() + targetLocation.getY(), new Rotation2d());

    SmartDashboard.putString("Starting Position", currentLocation.toString());
    SmartDashboard.putString("Target Position", calculatedPosition.toString());

    SmartDashboard.putNumber("Starting Angle", swerveSubsystem.getHeading());
    SmartDashboard.putNumber("Starting Heading", swerveSubsystem.getHeading());

    this.startingTime = Timer.getFPGATimestamp();

    this.xController.setSetpoint(calculatedPosition.getX());
    this.yController.setSetpoint(calculatedPosition.getY());
    this.thetaController.setSetpoint(swerveSubsystem.getYaw());
    this.startingHeading = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentLocation = swerveSubsystem.getEstimatedPose();
    double angle = swerveSubsystem.getYaw();

    double calculatedTheta = thetaController.calculate(angle);
    double calculatedX = xController.calculate(currentLocation.getX());
    double calculatedY = yController.calculate(currentLocation.getY());

    double thetaEffort = MathUtil.clamp(calculatedTheta, -0.5, 0.5);
    double xEffort = MathUtil.clamp(calculatedX, -speed, speed);
    double yEffort = MathUtil.clamp(calculatedY, -speed, speed);

    swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(new ChassisSpeeds(xEffort, yEffort, thetaEffort)));

    SmartDashboard.putNumber("thetaEffort", thetaEffort);
    SmartDashboard.putNumber("Theta Diff", thetaController.getPositionError());
    SmartDashboard.putNumber("xEffort", xEffort);
    SmartDashboard.putNumber("yEffort", yEffort);
    SmartDashboard.putNumber("Calculated Theta", calculatedTheta);
    SmartDashboard.putNumber("Calculated X", calculatedX);
    SmartDashboard.putNumber("Calculated Y", calculatedY);
    SmartDashboard.putNumber("Current Heading", angle);
    SmartDashboard.putNumber("xLocation Diff", xController.getPositionError());
    SmartDashboard.putNumber("yLocation Diff", yController.getPositionError());
    SmartDashboard.putNumber("xLocation", swerveSubsystem.getEstimatedPose().getX());
    SmartDashboard.putNumber("yLocation", swerveSubsystem.getEstimatedPose().getY());
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
    boolean finished = this.xController.atSetpoint() && this.yController.atSetpoint()
        && this.thetaController.atSetpoint();

    if (finished) {
      Telemetry.logData("---- DriveStraightByGyro ----",
          "Finished At: " + this.xController.getPositionError() + ", " + this.yController.getPositionError() + ", "
              + this.thetaController.getPositionError(),
          DriveByGyro.class);
    }

    return finished;
  }
}
