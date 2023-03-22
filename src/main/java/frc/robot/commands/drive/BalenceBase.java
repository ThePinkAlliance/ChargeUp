// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class BalenceBase extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  double pitchThreshold;
  double moveTime;
  double speed;

  Timer moveTimer;
  Timer pitchTimer;

  /** Creates a new BalenceBase. */
  public BalenceBase(double pitchThreshold, double speed, double moveTime, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.pitchThreshold = pitchThreshold;
    this.swerveSubsystem = swerveSubsystem;
    this.moveTime = moveTime;
    this.speed = speed;

    this.moveTimer = new Timer();
    this.pitchTimer = new Timer();

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchTimer.reset();
    pitchTimer.stop();
    moveTimer.reset();
    moveTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = swerveSubsystem.getPitch();

    if (currentPitch > pitchThreshold) {
      moveTimer.reset();
      moveTimer.start();

      swerveSubsystem
          .setModuleStates(
              Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(speed, 0, 0)));
    }

    if (currentPitch < -pitchThreshold) {
      moveTimer.reset();
      moveTimer.start();

      swerveSubsystem
          .setModuleStates(
              Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-speed, 0, 0)));
    }

    if (moveTimer.hasElapsed(moveTime)) {
      swerveSubsystem
          .setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
      moveTimer.stop();
      moveTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem
        .setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds()));

    pitchTimer.stop();
    moveTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPitch = swerveSubsystem.getPitch();

    if (currentPitch > -pitchThreshold && currentPitch < pitchThreshold) {
      pitchTimer.start();
    }

    return pitchTimer.hasElapsed(2);
  }
}
