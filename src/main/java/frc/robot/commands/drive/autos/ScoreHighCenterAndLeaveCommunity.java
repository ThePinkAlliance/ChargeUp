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
  public static Command scoreCubeHigh(ExtenderSubsystem extenderSubsystem, TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, SwerveSubsystem swerveSubsystem) {
    return UtilityCommands.pivotArm(127, armSubsystem).alongWith(new ExtendTicksPlus(86, 0.5, extenderSubsystem)).andThen(new GrabberOpen(grabberSubsystem, 1)).andThen(UtilityCommands.stow(armSubsystem, turretSubsystem, extenderSubsystem));
  }

  public static Command leaveCommunity(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem) {
    return new DriveStraightByGyro(-2, 1.8, false, swerveSubsystem).alongWith(UtilityCommands.pivotArm(210, armSubsystem)).andThen(new DriveByGyroInfinity(0, 0, 0, false, swerveSubsystem));
  }
}
