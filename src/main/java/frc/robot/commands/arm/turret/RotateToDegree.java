// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.turret;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

/**
 * Rotate command for the turret. Right now its configured with a dead reckon
 * controller however it is planned to switch over to pid.
 */
public class RotateToDegree extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean isFinished;
  private double desiredAngle;
  private double lastAngle;
  private double startingAngle;
  //private Supplier<Boolean> safeToContinue;
  double safetyPivotAngle;
  private Timer timer;
  private Timer epoch; //not sure the intention behind this timer so leaving it alone.  Added separate watchDogTimer.
  private PIDController controller;
  private CANSparkMax sparkMax;
  private ArmSubsystem armSubsystem;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(TurretSubsystem turretSubsystem, ArmSubsystem armSubsystem, double safetyPivotAngle, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
    this.lastAngle = 1;
    this.safetyPivotAngle = safetyPivotAngle;
    this.timer = new Timer();
    this.epoch = new Timer();
    
    
    // this.controller = new PIDController(2.3, 0.0121, 0);
    this.controller = new PIDController(0.1, 0.0, 0);
    this.controller.disableContinuousInput();

    this.sparkMax = turretSubsystem.getCanSparkMax();

    SmartDashboard.putNumber("turret-kP", controller.getP());
    SmartDashboard.putNumber("turret-kI", controller.getI());
    SmartDashboard.putNumber("turret-kD", controller.getD());
    
    //Do not require armSubsystem:  its only here to get information
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();

    epoch.start();
    epoch.reset();

    isFinished = false;
    startingAngle = turretSubsystem.getTurretAngle();

    double kP = SmartDashboard.getNumber("turret-kP", controller.getP());
    double kI = SmartDashboard.getNumber("turret-kI", controller.getI());
    double kD = SmartDashboard.getNumber("turret-kD", controller.getD());

    controller.setPID(kP, kI, kD);
    sparkMax.getPIDController().setP(kP);
    sparkMax.getPIDController().setI(kI);
    sparkMax.getPIDController().setD(kD);
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
      turretSubsystem.powerTurret(0);
      isFinished = true;
    }

    SmartDashboard.putNumber("Turret Target", desiredPosRadians);
    SmartDashboard.putNumber("Turret Position", currentAngle);

    epoch.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.powerTurret(0);
    timer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     
    return Math.abs(sparkMax.getEncoder().getPosition()) <= 0.3 || isFinished;
    
  }
}
