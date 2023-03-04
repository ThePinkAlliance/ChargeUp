package frc.robot.subsystems.motion;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

public abstract class Profile {
  private int numPoints = 0;
  private double[][] profile = {};
  private int timeoutMs = 0;
  private double targetRotations;

  public Profile(int numPoints, int timeoutMs, double targetRotations, double[][] profile) {
    this.numPoints = numPoints;
    this.profile = profile;
    this.timeoutMs = timeoutMs;
    this.targetRotations = targetRotations;
  }

  public double getDesiredRotations() {
    return this.targetRotations;
  }

  public int getNumberOfPoints() {
    return this.numPoints;
  }

  public int getTimeoutMs() {
    return this.timeoutMs;
  }

  public BufferedTrajectoryPointStream toTrajectory() {
    BufferedTrajectoryPointStream buff = new BufferedTrajectoryPointStream();

    /* This is fast since it's just into our TOP buffer */
    for (int i = 0; i < numPoints; ++i) {
      double positionRot = profile[i][0];
      double velocityRPM = profile[i][1];
      TrajectoryPoint point = new TrajectoryPoint();
      /* for each point, fill our structure and pass it to API */
      point.position = positionRot * 2048; // Convert Revolutions to Units
      point.velocity = velocityRPM * 2048 / 600.0; // Convert RPM to Units/100ms
      point.headingDeg = 0; /* future feature - not used in this example */
      point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
      point.profileSlotSelect1 = 0; /* future feature - not used in this example - cascaded PID [0,1], leave zero */
      point.timeDur = (int) profile[i][2];
      point.zeroPos = false;
      if (i == 0)
        point.zeroPos = true; /* set this to true on the first point */

      point.isLastPoint = false;
      if ((i + 1) == numPoints)
        point.isLastPoint = true; /* set this to true on the last point */

      buff.Write(point);
    }

    return buff;
  }
}
