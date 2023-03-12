// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import com.fasterxml.jackson.annotation.JsonProperty;

/** Add your docs here. */
public class AprilTagData {

  @JsonProperty("fID")
  public double fiducialID;

  @JsonProperty("fam")
  public String fiducialFamily;

  @JsonProperty("t6c_ts")
  private double[] cameraPose_targetSpace;

  @JsonProperty("t6r_fs")
  private double[] robotPose_fieldSpace;

  @JsonProperty("t6r_ts")
  private double[] robotPose_targetSpace;

  @JsonProperty("t6t_cs")
  private double[] targetPose_cameraSpace;

  @JsonProperty("t6t_rs")
  private double[] targetPose_robotSpace;

  @JsonProperty("ta")
  public double ta;

  @JsonProperty("tx")
  public double tx;

  @JsonProperty("txp")
  public double tx_pixels;

  @JsonProperty("ty")
  public double ty;

  @JsonProperty("typ")
  public double ty_pixels;

  @JsonProperty("ts")
  public double ts;

  public AprilTagData() {
    cameraPose_targetSpace = new double[6];
    robotPose_fieldSpace = new double[6];
    robotPose_targetSpace = new double[6];
    targetPose_cameraSpace = new double[6];
    targetPose_robotSpace = new double[6];
  }

  public double[] getRobotPose_fieldSpace() {
    return this.robotPose_fieldSpace;
  }

  public double[] getRobotPose_targetSpace() {
    return this.robotPose_targetSpace;
  }

  public double[] getCameraPose_targetSpace() {
    return this.cameraPose_targetSpace;
  }
}
