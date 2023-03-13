// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;
import frc.robot.Telemetry;

/**
 * Rotate command for the turret. Right now its configured with a dead reckon
 * controller however it is planned to switch over to pid.
 */
public class RotateToDegree extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean isFinished;
  private double desiredAngle;
  double safetyPivotAngle;
  private CANSparkMax sparkMax;
  private ArmSubsystem armSubsystem;
  private double angleTolerence;
  Watchdog watchdog;
  private final double WATCHDOG_TIMEOUT = 4.2;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem, double safetyPivotAngle, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
    this.safetyPivotAngle = safetyPivotAngle;
    this.watchdog = new Watchdog(WATCHDOG_TIMEOUT, () -> {
      //empty on purpose, end() will handle safing the subsystem
    });
    this.sparkMax = turretSubsystem.getCanSparkMax();
    
    //Do not require armSubsystem:  its only here to get information
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   
    

    isFinished = false;

    sparkMax.getPIDController().setP(0.1);
    sparkMax.getPIDController().setI(0);
    sparkMax.getPIDController().setD(0);
    watchdog.reset();
    watchdog.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle = this.turretSubsystem.getTurretAngle() * (Math.PI / 180);
    double desiredPosRadians = desiredAngle * (Math.PI / 180);
    double desiredRotations = desiredPosRadians * (348.7 / (2 * Math.PI));
    System.out.println("armSubSystem.getArmPitch() " + armSubsystem.getArmPitch());
    if (armSubsystem.getArmPitch() > safetyPivotAngle) {
       sparkMax.getPIDController().setReference(desiredRotations, ControlType.kPosition);
    } else {
      isFinished = true;
    }
    Telemetry.logData("Turret Target Angle", desiredPosRadians, RotateToDegree.class);
    Telemetry.logData("Current Turret Angle", currentAngle, RotateToDegree.class);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDesired = this.turretSubsystem.getTurretAngle() * (Math.PI / 180);
    double difference = Math.abs(desiredAngle - currentDesired);
    return difference <= angleTolerence || isFinished || watchdog.isExpired(); 
  }
}
