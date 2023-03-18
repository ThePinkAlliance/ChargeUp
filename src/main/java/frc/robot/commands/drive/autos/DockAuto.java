// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;

public class DockAuto extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  private double angleRange;
  private boolean hasReachedDock;
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
    this.hasReachedDock = false;
    this.isFinished = true;

    this.watchdog = new Watchdog(4, () -> {
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
    this.hasReachedDock = false;
    this.isFinished = true;

    this.swerveSubsystem.setModuleStates(
        Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(MAX_POWER_METERS, 0, 0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (swerveSubsystem.getPitch() >= 10) {
      hasReachedDock = true;

      // watchdog.;
    }

    if (hasReachedDock) {
      // double power =
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
