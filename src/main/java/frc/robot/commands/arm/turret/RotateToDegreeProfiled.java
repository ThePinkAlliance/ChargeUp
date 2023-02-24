// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.arm.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToDegreeProfiled extends TrapezoidProfileCommand {

  /** Creates a new RotateToDegreeProfiled. */
  public RotateToDegreeProfiled(TurretSubsystem turretSubsystem, double desiredAngle) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(20, 12),
            // Goal state
            new TrapezoidProfile.State(desiredAngle, 0),
            // Initial state
            new TrapezoidProfile.State(turretSubsystem.getTurretAngle(), 10)),
        state -> {
          double maxVel = 30;
          double d = 0;
          double w = 1.1 / maxVel;
          double r = w * Math.sqrt(Math.pow(state.velocity, 2) - Math.pow(d, 2)) * -1;
          // Use current trajectory state here
          SmartDashboard.putNumber("profile pos", state.position);
          SmartDashboard.putNumber("profile vel", state.velocity);
          SmartDashboard.putNumber("power", r);
          SmartDashboard.putNumber("currentAngle", turretSubsystem.getTurretAngle());
          SmartDashboard.putNumber("achieveableAngle", desiredAngle);
          turretSubsystem.powerTurret(r);
        });

  }
}
