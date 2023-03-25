// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.ExtenderSubsystem;

public class JoystickArmExtend extends CommandBase {
  private ExtenderSubsystem extenderSubsystem;
  private Supplier<Double> extSupplier;

  Joystick joystick;

  /** Creates a new CommandExtend. */
  public JoystickArmExtend(Joystick joystick, ExtenderSubsystem extenderSubsystem, Supplier<Double> extSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = joystick;
    this.extenderSubsystem = extenderSubsystem;
    this.extSupplier = extSupplier;
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double val = extSupplier.get();
    // Cube law on extended input
    // val = val * Math.abs(val);
    val = val * val * val;
    if (joystick.getRawButton(Constants.OIConstants.kButtonStart)) {
      this.extenderSubsystem.disableExtendReverseSoftLimits();
    } else {
      this.extenderSubsystem.enableExtendReverseSoftLimits();
    }
    this.extenderSubsystem.commandExtend(val * -1);

    SmartDashboard.putNumber("Extend Current", extenderSubsystem.getExtendCurrent());
    SmartDashboard.putNumber("Extend Position", extenderSubsystem.getExtendedPosition());
    SmartDashboard.putNumber("Extend Distance", extenderSubsystem.getExtensionDistance());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.extenderSubsystem.commandExtend(0);
    Telemetry.logData("Status", "Ended", JoystickArmExtend.class);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
