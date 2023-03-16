// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.Navigate;
import frc.robot.commands.drive.TimedNavigate;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndLeaveCommunity extends SequentialCommandGroup {
  /** Creates a new ScoreAndLeaveCommunity. */
  public ScoreAndLeaveCommunity(SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TimedNavigate(swerveSubsystem, new ChassisSpeeds(-2,
        0, 0), .7),
        new Navigate(swerveSubsystem, new SwerveModulePosition(
            2.89, new Rotation2d()), 1.2));
  }
}
