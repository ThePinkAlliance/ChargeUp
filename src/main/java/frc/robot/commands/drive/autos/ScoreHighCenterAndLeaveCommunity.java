// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.UtilityCommands;
import frc.robot.commands.drive.DriveByGyroInfinity;
import frc.robot.commands.drive.DriveStraightByGyro;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;

/** Add your docs here. */
public class ScoreHighCenterAndLeaveCommunity {
  /*
   * Be careful of the order in which parallel command groups are executed because
   * when the command at the end of the group ends all the rest end even if they
   * are done.
   */
  public static Command leaveCommunityCenter(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem) {
    return UtilityCommands.pivotArm(200, armSubsystem)
        .alongWith(new DriveStraightByGyro(-2, 3, false, swerveSubsystem))
        .andThen(
            UtilityCommands.pivotArm(160, armSubsystem).alongWith(new DriveByGyroInfinity(-1.8, -8, 1.8, false,
                swerveSubsystem)))
        .andThen(new DriveStraightByGyro(-1.6, 3, swerveSubsystem));
  }

  public static Command leaveCommunityRight(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem) {
    return UtilityCommands.pivotArm(200, armSubsystem)
        .alongWith(new DriveStraightByGyro(-2, 1.8, false, swerveSubsystem))
        .andThen(UtilityCommands.pivotArm(160,
            armSubsystem).alongWith(
                new DriveByGyroInfinity(-1.8, -8, 1.5, false,
                    swerveSubsystem)))
        .andThen(new DriveStraightByGyro(-1.6, 1.8, swerveSubsystem));
  }

  public static Command balanceStation(SwerveSubsystem swerveSubsystem) {
    return new DockAuto(swerveSubsystem, 0, 2, 37, 3.0);
  }
}
