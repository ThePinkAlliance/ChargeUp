// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveStraightByGyro;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TAutoScoreCubeLeftLeave extends SequentialCommandGroup {
  /** Creates a new TAutoScoreCubeLeftLeave. */
  public TAutoScoreCubeLeftLeave(SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double distanceToDeliver = 2;
    double power = 3;
    addCommands(new DriveStraightByGyro(power, distanceToDeliver, swerveSubsystem));
  }
}