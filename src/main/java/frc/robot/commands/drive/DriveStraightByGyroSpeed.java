// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
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

public class DriveStraightByGyroSpeed extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  PIDController thetaController;
  PIDController xController;

  double distance;
  double startingTime;
  double startingHeading;
  double speed;
  boolean doStop;

  TrapezoidProfile.State lastRef;
  double lastEpoch;
  double target;

  /** Creates a new DriveStraightByGyro. */
  public DriveStraightByGyroSpeed(double distance, double speed, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.thetaController = new PIDController(.4, 0.4, 0);
    this.xController = new PIDController(4, 0, 0.5);

    /*
     * The controller needs an I gain later.
     */
    this.xController.setTolerance(0.155);

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

  public DriveStraightByGyroSpeed(double distance, double speed, boolean doStop, SwerveSubsystem swerveSubsystem) {
    this(distance, speed, swerveSubsystem);

    this.doStop = doStop;
  }

  public DriveStraightByGyroSpeed configureTolerence(double tol) {
    this.xController.setTolerance(tol);

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.lastEpoch = Timer.getFPGATimestamp();
    this.lastRef = new State();

    double xLocation = swerveSubsystem.getEstimatedPose().getX();
    double xLocationTarget = distance + xLocation;

    target = distance + xLocation;

    SmartDashboard.putNumber("Starting Position", xLocation);
    SmartDashboard.putNumber("Target Position", xLocationTarget);
    SmartDashboard.putNumber("Starting Angle", swerveSubsystem.getHeading());
    SmartDashboard.putNumber("Starting Heading", swerveSubsystem.getHeading());

    this.startingTime = Timer.getFPGATimestamp();

    this.xController.setSetpoint(xLocationTarget);
    this.thetaController.setSetpoint(swerveSubsystem.getYaw());
    this.startingHeading = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile profile = new TrapezoidProfile(new Constraints(speed, 12),
        new State(target, 0), lastRef);
    TrapezoidProfile.State state = profile.calculate(lastEpoch - Timer.getFPGATimestamp());

    double xLocation = swerveSubsystem.getEstimatedPose().getX();
    double angle = swerveSubsystem.getYaw();
    double calculatedTheta = thetaController.calculate(angle);
    double calculatedX = xController.calculate(xLocation, state.position);
    double thetaEffort = MathUtil.clamp(calculatedTheta, -0.5, 0.5);
    double xEffort = MathUtil.clamp(calculatedX, -speed, speed);

    swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics
            .toSwerveModuleStates(new ChassisSpeeds(xEffort, 0, thetaEffort)));

    SmartDashboard.putNumber("Motion Profile Out", state.position);
    SmartDashboard.putNumber("thetaEffort", thetaEffort);
    SmartDashboard.putNumber("Theta Diff", thetaController.getPositionError());
    SmartDashboard.putNumber("xEffort", xEffort);
    SmartDashboard.putNumber("Calculated Theta", calculatedTheta);
    SmartDashboard.putNumber("Calculated X", calculatedX);
    SmartDashboard.putNumber("Current Heading", angle);
    SmartDashboard.putNumber("Location Diff", xController.getPositionError());
    SmartDashboard.putNumber("xLocation", swerveSubsystem.getEstimatedPose().getX());

    lastRef = state;
    lastEpoch = Timer.getFPGATimestamp();
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
          DriveStraightByGyroSpeed.class);
    }

    return finished;
  }
}
