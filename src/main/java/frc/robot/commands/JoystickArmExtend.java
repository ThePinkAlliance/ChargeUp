// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.ArmSubsystem;

public class JoystickArmExtend extends CommandBase {
  private ArmSubsystem armSubsystem;
  private Supplier<Double> extSupplier;

  Joystick joystick;

  /** Creates a new CommandExtend. */
  public JoystickArmExtend(Joystick joystick, ArmSubsystem armSubsystem, Supplier<Double> extSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = joystick;
    this.armSubsystem = armSubsystem;
    this.extSupplier = extSupplier;
    addRequirements(armSubsystem);
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
      this.armSubsystem.disableExtendReverseSoftLimits();
    } else {
      this.armSubsystem.enableExtendReverseSoftLimits();
    }
    this.armSubsystem.commandExtend(val * -1);

    SmartDashboard.putNumber("Extend Current", armSubsystem.getExtendCurrent());
    SmartDashboard.putNumber("Extend Position", armSubsystem.getExtendedPosition());
    SmartDashboard.putNumber("Extend Distance", armSubsystem.getExtensionDistance());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.commandExtend(0);
    Telemetry.logData("Status", "Ended", JoystickArmExtend.class);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
