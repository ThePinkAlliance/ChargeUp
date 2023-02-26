package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Translation3d;

public class Node {
  public enum ObjectType {
    CONE,
    CUBE
  }

  public enum Level {
    LOW,
    MID,
    HIGH
  }

  ObjectType type;
  Translation3d location;
  Level level;

  public Node(ObjectType type, Level level, Translation3d location) {
    this.type = type;
    this.location = location;
    this.level = level;
  }

  public Level getLevel() {
    return level;
  }

  public ObjectType getObjectType() {
    return type;
  }

  public Translation3d getLocation() {
    return location;
  }

  public double distanceTo(Node node) {
    return this.location.getDistance(node.location);
  }
}
