// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;
import frc.robot.subsystems.camera.CameraData;
import frc.robot.subsystems.camera.CameraData.TargetData;
import frc.robot.subsystems.camera.CameraInterface.PipelineType;
import frc.robot.Constants;
import frc.robot.Telemetry;

/**
 * Rotate command for the turret. Right now its configured with a dead reckon
 * controller however it is planned to switch over to pid.
 */
public class RotateBasedOnExternalSensor extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private CameraSubsystem cameraSubsystem;
  private boolean isFinished;
  private double desiredRotations;
  private double startingPosition;
  private double desiredPosition;
  private double error;
  private double safetyPivotAngle;
  private CANSparkMax sparkMax;
  private ArmSubsystem armSubsystem;
  private PipelineType pType;
  Watchdog watchdog;
  private final double WATCHDOG_TIMEOUT = 0.5;

  /** Creates a new RotateBasedOnExternalSensor. */
  public RotateBasedOnExternalSensor(TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem, CameraSubsystem cameraSubsystem, double safetyPivotAngle, PipelineType pType) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.armSubsystem = armSubsystem;
    this.cameraSubsystem = cameraSubsystem;
    this.safetyPivotAngle = safetyPivotAngle;
    this.pType = pType;
    this.watchdog = new Watchdog(WATCHDOG_TIMEOUT, () -> {
      // empty on purpose, end() will handle safing the subsystem
    });
    this.sparkMax = turretSubsystem.getCanSparkMax();

    // Do not require armSubsystem: its only here to get information
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    cameraSubsystem.setPipeline(pType);
    //may need wait here
    startingPosition = turretSubsystem.getTurretPosition();
    error = getAngleErrorFromRetro();
    desiredRotations = (-error * 350)/360;
    desiredPosition = startingPosition + desiredRotations;
    Telemetry.logData("Starting Position:", startingPosition, RotateBasedOnExternalSensor.class);
    Telemetry.logData("Desired Rotations:", desiredRotations, RotateBasedOnExternalSensor.class);
    Telemetry.logData("Sensor Error:", error, RotateBasedOnExternalSensor.class);
    sparkMax.getPIDController().setP(Constants.TurretConstants.PID_ROTATE_GAINS_FX.kP);
    sparkMax.getPIDController().setI(Constants.TurretConstants.PID_ROTATE_GAINS_FX.kI);
    sparkMax.getPIDController().setD(Constants.TurretConstants.PID_ROTATE_GAINS_FX.kD);
    watchdog.reset();
    watchdog.enable();
    System.out.println("armSubSystem.getArmPitch() " + armSubsystem.getArmPitch());
    if (armSubsystem.getArmPitch() > safetyPivotAngle) {
      Telemetry.logData("Commanding: ", desiredRotations, RotateBasedOnExternalSensor.class);
      sparkMax.getPIDController().setReference(desiredRotations, CANSparkMax.ControlType.kPosition);
    } else {
      isFinished = true;
    }
  }

  // Called every time the s cheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    System.out.println("armSubSystem.getArmPitch() " + armSubsystem.getArmPitch());
    if (armSubsystem.getArmPitch() > safetyPivotAngle) {
      Telemetry.logData("Commanding: ", desiredRotations, RotateBasedOnExternalSensor.class);
      sparkMax.getPIDController().setReference(desiredRotations, ControlType.kPosition);
    } else {
      isFinished = true;
    }*/
    Telemetry.logData("Turret Target Angle", desiredRotations, RotateBasedOnExternalSensor.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    double difference = Math.abs(turretSubsystem.getTurretPosition() - desiredPosition);
    boolean toleranceMet = (difference <= Constants.TurretConstants.ANGLE_TOLERANCE_FOR_EXTERNAL_SENSOR);
    Telemetry.logData("Current Difference", difference, RotateBasedOnExternalSensor.class);
    Telemetry.logData("isFinished", isFinished, RotateBasedOnExternalSensor.class);
    Telemetry.logData("toleranceMet", toleranceMet, RotateBasedOnExternalSensor.class);
    
    return toleranceMet || isFinished || watchdog.isExpired();
  }

  private double getAngleErrorFromRetro() {

    CameraData camResult = cameraSubsystem.getTarget();
    double error = 0.0;
    if (camResult.pipelineType == PipelineType.REFLECTIVE_HIGH) {
        if (camResult.hasTargets()) {
            Telemetry.logData("Has Targets", camResult.getTargets().get(0), getClass());
            TargetData target = camResult.getTargets().get(0);
            double x = camResult.getTargets().get(0).targetXAngle;
            double y = camResult.getTargets().get(0).targetYAngle;
            Telemetry.logData("VX", x, getClass());
            Telemetry.logData("VY", y, getClass());
            error = x;
        }   
    }
    return error;
}
  
}
