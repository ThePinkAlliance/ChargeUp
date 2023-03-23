// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.UtilityCommands;
import frc.robot.commands.arm.extend.ExtendTicksPlus;
import frc.robot.commands.arm.grabber.GrabberOpen;
import frc.robot.commands.drive.DriveByGyroInfinity;
import frc.robot.commands.drive.DriveStraightByGyro;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ExtenderSubsystem;
import frc.robot.subsystems.arm.GrabberSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

/** Add your docs here. */
public class ScoreHighCenterAndLeaveCommunity {
  public static Command leaveCommunityCenter(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem) {
    return new DriveStraightByGyro(-2, 2.2, false, swerveSubsystem)
        .alongWith(UtilityCommands.pivotArm(200, armSubsystem))
        .andThen(new DriveByGyroInfinity(-1.8, -8, 1.8, false, swerveSubsystem)
            .alongWith(UtilityCommands.pivotArm(160, armSubsystem)))
        .andThen(new DriveStraightByGyro(-1.6, 2.2, swerveSubsystem));
  }

  public static Command leaveCommunityRight(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem) {
    return new DriveStraightByGyro(-2, 1.8, false, swerveSubsystem)
        .alongWith(UtilityCommands.pivotArm(200, armSubsystem))
        .andThen(new DriveByGyroInfinity(-1.8, -8, 1.5, false, swerveSubsystem)
            .alongWith(UtilityCommands.pivotArm(160, armSubsystem)))
        .andThen(new DriveStraightByGyro(-1.6, 1.8, swerveSubsystem));
  }

  public static Command balanceStation(SwerveSubsystem swerveSubsystem) {
    return new DockAuto(swerveSubsystem, 0, 2, 37, 3.0);
  }
}
