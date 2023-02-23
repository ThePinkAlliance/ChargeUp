package frc.robot.subsystems.scoring;

import java.util.ArrayList;

public class NodeHelper {
  private static double xDistance = 23;
  private static double yDistance = 23;

  public static ArrayList<Node> createRow(int objectAmount) {
    ArrayList<Node> collection = new ArrayList<>();

    /**
     * Create the row of scoring locations with the appropriate data.
     */
    for (int i = 0; i < objectAmount; i++) {
      double x = xDistance * (i + 1);
      double y = yDistance * (i + 1);
    }

    return collection;
  }
}
