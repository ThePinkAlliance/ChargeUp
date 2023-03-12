// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.network.GridSubscriber;

public class ScoringSubsystem extends SubsystemBase {
  private final int EXTEND = 0;
  private final int PITCH = 1;
  private final int TURRET = 2;
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

  private GridSubscriber m_gridSubscriber;

  /** Creates a new ScoringSubsystem. */
  public ScoringSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get a topic from a NetworkTableInstance
    // the topic name in this case is the full name
    DoubleTopic dblTopic = inst.getDoubleTopic("/datatable/CUI");
    m_gridSubscriber = new GridSubscriber(dblTopic);
  }

  public double[] getPositionData(int index) {
    // if (index > positionData.length) {
    // return positionData[0];
    // }

    return positionData[index];
  }

  public double getPositionData_Extend() {
    double value = 0.0;
    int index = (int)m_gridSubscriber.getGridTarget();
    if (index >= 0 && index < positionData.length) {
      value = positionData[index][EXTEND];
    }
    System.out.println("Grid Target Selected INDEX/EXTEND: " + index + "/" + value);
    return value;
  }
  public double getPositionData_Pitch() {
    double value = 0.0;
    int index = (int)m_gridSubscriber.getGridTarget();
    if (index >= 0 && index < positionData.length) {
      value = positionData[index][PITCH];
    }
    System.out.println("Grid Target Selected INDEX/PITCH : " + index + "/" + value);
    return value;
  }
  public double getPositionData_Turret() {
    double value = 0.0;
    int index = (int)m_gridSubscriber.getGridTarget();
    if (index >= 0 && index < positionData.length) {
      value = positionData[index][TURRET];
    }
    System.out.println("Grid Target Selected INDEX/TURRET: " + index + "/" + value);
    return value;
  }

  @Override
  public void periodic() {
    //Call the subscriber periodic
    m_gridSubscriber.periodic();
  }
}
