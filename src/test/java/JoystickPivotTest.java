import org.junit.jupiter.api.Test;

public class JoystickPivotTest {
  double x = 1;
  double theta = 270;
  double speed = 0.2;

  @Test
  public void run() {
    double controlEffort = x * theta <= 180 ? speed : -speed;

    System.out.println(controlEffort);
  }
}
