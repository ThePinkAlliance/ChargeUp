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

public class StrafeByGyro extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  PIDController thetaController;
  PIDController yController;

  double distance;
  double startingTime;
  double startingHeading;
  double speed;
  boolean doStop;

  boolean enableDebug;

  /** Creates a new DriveStraightByGyro. */
  public StrafeByGyro(double distance, double speed, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.thetaController = new PIDController(.2, 0, 0);
    this.yController = new PIDController(4.6, 0.005, 0.025);
    this.enableDebug = false;

    /*
     * The controller needs an I gain later.
     */
    this.yController.setTolerance(0.15);
    this.thetaController.setTolerance(.5);

    this.speed = speed;
    this.doStop = true;
    this.distance = distance;

    addRequirements(swerveSubsystem);
  }

  public StrafeByGyro debug() {
    this.enableDebug = true;

    return this;
  }

  public StrafeByGyro(double distance, double speed, boolean doStop, SwerveSubsystem swerveSubsystem) {
    this(distance, speed, swerveSubsystem);

    this.doStop = doStop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double yLocation = swerveSubsystem.getEstimatedPose().getY();
    double yLocationTarget = distance + yLocation;

    if (enableDebug) {
      SmartDashboard.putNumber("Starting Position", yLocation);
      SmartDashboard.putNumber("Target Position", yLocationTarget);
      SmartDashboard.putNumber("Starting Angle", swerveSubsystem.getHeading());
    }

    this.startingTime = Timer.getFPGATimestamp();

    this.yController.setSetpoint(yLocationTarget);
    this.thetaController.setSetpoint(swerveSubsystem.getHeading());
    this.startingHeading = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yLocation = swerveSubsystem.getEstimatedPose().getY();
    double angle = swerveSubsystem.getHeading();
    double thetaEffort = MathUtil.clamp(thetaController.calculate(angle), -speed, speed);
    double yEffort = MathUtil.clamp(yController.calculate(yLocation), -speed, speed);

    swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(new ChassisSpeeds(0, yEffort, thetaEffort)));

    if (enableDebug) {
      SmartDashboard.putNumber("thetaEffort", thetaEffort);
      SmartDashboard.putNumber("yEffort", yEffort);
      SmartDashboard.putNumber("yLocation Diff", yController.getPositionError());
      SmartDashboard.putNumber("yLocation", yLocation);
    }
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
    boolean finished = this.yController.atSetpoint();

    if (finished) {
      Telemetry.logData("---- StrafeByGyro ----", "Finished At: " + this.yController.getPositionError(),
          StrafeByGyro.class);
    }

    return finished;
  }
}
