// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.grabber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.GrabberSubsystem;

public class JoystickGrabber extends CommandBase {
  GrabberSubsystem grabberSubsystem;
  Supplier<Double> intake;
  Supplier<Double> grasp;

  /** Creates a new JoystickGrabber. */
  public JoystickGrabber(Supplier<Double> intake, Supplier<Double> grasp, GrabberSubsystem grabberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.grasp = grasp;

    this.grabberSubsystem = grabberSubsystem;

    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakeInput = intake.get();
    double graspInput = grasp.get();
    this.grabberSubsystem.setIntakeSpeed(intake.get());
    this.grabberSubsystem.setGraspPower(grasp.get());
    SmartDashboard.putNumber("Grabber.Grasp.Input", graspInput);
    SmartDashboard.putNumber("Grabber.Grasp.Current", grabberSubsystem.getGraspCurrent());
    SmartDashboard.putNumber("Grabber.Intake.Input", intakeInput);
    SmartDashboard.putNumber("Grabber.Intake.Current", grabberSubsystem.getIntakeCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.grabberSubsystem.setIntakeSpeed(0);
    this.grabberSubsystem.setGraspPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
