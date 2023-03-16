// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TimedNavigate extends CommandBase {
  private ChassisSpeeds speeds;
  private SwerveSubsystem swerveSubsystem;
  private Timer timer;
  private double time;

  /** Creates a new TImedNavigate. */
  public TimedNavigate(SwerveSubsystem swerveSubsystem, ChassisSpeeds speeds, double time) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.speeds = speeds;
    this.time = time;
    this.swerveSubsystem = swerveSubsystem;
    this.timer = new Timer();

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem
        .setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
