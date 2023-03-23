// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;

public class DockAuto extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  private double angleRange;
  private boolean didReachDock;
  private boolean isFinished;

  private final double MAX_POWER_METERS;
  private final double MAX_ANGLE;
  private final double APPROACH_SPEED;

  private final Watchdog watchdog;

  /** Creates a new DockAuto. */
  public DockAuto(SwerveSubsystem swerveSubsystem, double angleRange, double MAX_POWER_METERS, double MAX_ANGLE,
      double APPROACH_SPEED) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;
    this.angleRange = angleRange;
    this.didReachDock = false;
    this.isFinished = false;

    this.watchdog = new Watchdog(10, () -> {
      this.swerveSubsystem
          .setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds()));

      this.isFinished = true;
    });

    this.MAX_POWER_METERS = MAX_POWER_METERS;
    this.APPROACH_SPEED = APPROACH_SPEED;
    this.MAX_ANGLE = MAX_ANGLE;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.didReachDock = false;
    this.isFinished = false;
    this.watchdog.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = swerveSubsystem.getPitch();
    double approachPitchThreshold = 12;
    double pitchSettle = 12;
    // 14 might be too high.
    if (currentPitch >= approachPitchThreshold && !didReachDock) {
      didReachDock = true;

      watchdog.reset();
    } else if (!didReachDock) {
      this.swerveSubsystem.setModuleStates(
          Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(APPROACH_SPEED, 0, 0)));
    }

    if (didReachDock) {
      double gain = 0.062;
      double power = gain * currentPitch;

      /* Power ceiling and floor */
      /*
       * if (Math.abs(currentPitch) > MAX_ANGLE) {
       * power = Math.copySign((MAX_POWER_METERS / MAX_ANGLE) * MAX_ANGLE,
       * currentPitch);
       * } else if (Math.abs(currentPitch) < pitchFloor) {
       * power = 0;
       * 
       * isFinished = true;
       * }
       */

      if (Math.abs(currentPitch) < pitchSettle) {
        power = 0;
        isFinished = true;
      }

      swerveSubsystem.setModuleStates(
          Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(power, 0, 0)));
    }
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
    boolean didFinish = isFinished;
    boolean watchdogExpired = watchdog.isExpired();

    if (didFinish) {
      Telemetry.logData("---- Dock Auto ----; didFinish", didFinish, DockAuto.class);
    }

    if (watchdogExpired) {
      Telemetry.logData("---- Dock Auto ----; watchdog expired", watchdogExpired, DockAuto.class);
    }

    return didFinish || watchdogExpired;
  }
}
