// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.arm.ArmSubsystem;

public class JoystickArm extends CommandBase {
  private ArmSubsystem armSubsystem;
  private Supplier<Double> extSupplier;
  private Supplier<Double> pivotSupplier;
  private boolean updateHoldPosition;
  private final double PIVOT_DEADBAND = 0.05;
  private double ANGLE_FLOOR;
  private double ANGLE_CEILING;

  /** Creates a new CommandExtend. */
  public JoystickArm(ArmSubsystem armSubsystem, Supplier<Double> extSupplier, Supplier<Double> pivotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.extSupplier = extSupplier;
    this.pivotSupplier = pivotSupplier;
    this.updateHoldPosition = false;
    this.ANGLE_FLOOR = 75;
    this.ANGLE_CEILING = 218;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateHoldPosition = false;
    armSubsystem.getPivotTalon().configFactoryDefault();
    armSubsystem.getPivotTalon().config_kP(0, 0.1);
    this.armSubsystem.setPositionToHold(this.armSubsystem.getPivotTalon().getSelectedSensorPosition());
    Telemetry.logData("Status", "Init", JoystickArm.class);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = Math.abs(pivotSupplier.get()) > PIVOT_DEADBAND ? pivotSupplier.get() : 0;
    double pivotAngle = armSubsystem.getArmPitch();

    if (input == 0) {
      if (updateHoldPosition) {
        this.armSubsystem.setPositionToHold(this.armSubsystem.getPivotTalon().getSelectedSensorPosition());

        updateHoldPosition = false;
      }

      // this.armSubsystem.getPivotTalon().set(ControlMode.Position,
      // armSubsystem.getPositionToHold());
      // Telemetry.logData("Position To Hold", armSubsystem.getPositionToHold(),
      // JoystickArm.class);
    } else {
      if ((this.armSubsystem.getArmPitch() < ANGLE_FLOOR && Math.signum(input) == 1
          || this.armSubsystem.getArmPitch() > ANGLE_FLOOR && Math.signum(input) == -1)
          || (this.armSubsystem.getArmPitch() > ANGLE_CEILING && Math.signum(input) == -1
              || this.armSubsystem.getArmPitch() < ANGLE_CEILING && Math.signum(input) == 1)) {
        this.armSubsystem.commandPivot(input);
      } else {
        this.armSubsystem.commandPivot(0);
      }

      this.updateHoldPosition = true;
    }

    double val = extSupplier.get();
    // Cube law on extended input
    // val = val * Math.abs(val);
    val = val * val * val;
    this.armSubsystem.commandExtend(val * -1);

    SmartDashboard.putNumber("Pivot Demanded Power", armSubsystem.getPivotDemandedPower());
    SmartDashboard.putNumber("Pivot Power", input);
    SmartDashboard.putNumber("Pivot Angle", pivotAngle);

    SmartDashboard.putNumber("Extend Current", armSubsystem.getExtendCurrent());
    SmartDashboard.putNumber("Extend Position", armSubsystem.getExtendedPosition());
    SmartDashboard.putNumber("Extend Distance", armSubsystem.getExtensionDistance());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.commandPivot(0);
    Telemetry.logData("Status", "Ended", JoystickArm.class);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
