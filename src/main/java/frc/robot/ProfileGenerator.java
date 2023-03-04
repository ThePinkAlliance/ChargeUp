// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

/** Add your docs here. */
public class ProfileGenerator {
  private double n;
  private double dist;
  private double vProg;
  private double itp;
  private double fl1;
  private double fl2;
  private double t4;
  private double t1;
  private double t2;

  private double[][] points;

  public ProfileGenerator(double dist, double t1, double t2, double vProg, double itp) {
    this.dist = dist;
    this.t1 = t1;
    this.t2 = t2;
    this.itp = itp;
    this.vProg = vProg;

    this.fl1 = Math.round(t1 / itp);
    this.fl2 = Math.round(t2 / itp);
    this.t4 = dist / vProg;
    this.n = t4 / itp;
  }

  private BufferedTrajectoryPointStream generatePoints() {
    BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
    ArrayList<MotionPoint> points = new ArrayList<>();

    int accumSteps = 0;

    for (int i = 0; i < 399; i++) {
      TrajectoryPoint point = new TrajectoryPoint();

      int idx = i > 0 ? i + 1 : 0;
      int input = idx < n + 2 ? 1 : 0;
      double lastF1 = i > 0 ? points.get(i - 1).f1 : 0;
      double f1 = Math.max(0, Math.min(1, lastF1 + input == 1 ? 1 / this.fl1 : -1 / this.fl1));

      // sussy. lookup how to use offset.
      double f2 = f1 + accumSteps;

      point.timeDur = (int) (idx * itp / 1000);

      accumSteps += (i + 1);
    }

    return stream;
  }
}

class MotionPoint {
  public int step;
  public int input;
  public double time;
  public double f1;
  public double f2;
  public boolean outputIncluded;
  public double outputVel;
  public double outputPos;
  public double outputAcc;
  public boolean isFirst;
  public boolean isLast;
  public int oneIfZero;

  public MotionPoint(int step, int input, double time, double f1, double f2, boolean outputIncluded, double outputVel,
      double outputPos, double outputAcc, boolean isFirst, boolean isLast, int oneIfZero) {
    this.f1 = f1;
    this.f2 = f2;
    this.step = step;
    this.input = input;
    this.time = time;
    this.outputIncluded = outputIncluded;
    this.outputVel = outputVel;
    this.outputPos = outputPos;
    this.outputAcc = outputAcc;
    this.isFirst = isFirst;
    this.isLast = isLast;
    this.oneIfZero = oneIfZero;
  }

}