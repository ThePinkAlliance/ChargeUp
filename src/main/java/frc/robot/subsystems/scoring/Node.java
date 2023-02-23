package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Translation2d;

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
  Translation2d location;
  Level level;

  public Node(ObjectType type, Translation2d location, Level level) {
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

  public Translation2d getLocation() {
    return location;
  }
}
