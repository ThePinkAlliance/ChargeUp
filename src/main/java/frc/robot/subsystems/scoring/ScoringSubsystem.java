// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
  /* Extend Position | Arm Pitch | Turret Angle */
  private double[][] positionData = {
      { 30, 82, 0 }, // 0
      { 30, 126, -27 }, // 1
      { 30, 126, 0 }, // 2
      { 30, 126, 27 }, // 3
      { 30, 126, -23 }, // 4
      { 30, 126, 0 }, // 5
      { 30, 126, 23 }, // 6
      { 30, 126, -23 }, // 7
      { 30, 126, 0 }, // 8
      { 30, 126, 23 } // 9
  };

  /** Creates a new ScoringSubsystem. */
  public ScoringSubsystem() {
  }

  public double[] getPositionData(int index) {
    if (index > positionData.length || index < 0) {
      return positionData[0];
    }

    return positionData[index];
  }

  @Override
  public void periodic() {
  }
}
