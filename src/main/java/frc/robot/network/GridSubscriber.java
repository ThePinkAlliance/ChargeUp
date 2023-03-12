package frc.robot.network;

import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GridSubscriber {
  // the publisher is an instance variable so its lifetime matches that of the
  // class
  private DoubleSubscriber dblSub;
  private double gridTarget = 0;

  public GridSubscriber(DoubleTopic dblTopic) {
    // subscribe options may be specified using PubSubOption
    dblSub = dblTopic.subscribe(99.0, PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
  }

  public double getGridTarget() {
    return gridTarget;
  }

  public void periodic() {
    // simple get of most recent value; if no value has been published,
    // returns the default value passed to the subscribe() function
    gridTarget = dblSub.get();

    SmartDashboard.putNumber("GridPublisher CUI", gridTarget);
    gridderUpdater(gridTarget);
  }

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop publishing
  public void close() {
    // stop publishing
    dblSub.close();
  }

  public Supplier<Integer> getTargetSupplier() {
    return () -> (int) gridTarget;
  }

  public void dashBoardGridder() {
    SmartDashboard.putBoolean("zero", false);
    SmartDashboard.putBoolean("one", false);
    SmartDashboard.putBoolean("two", false);
    SmartDashboard.putBoolean("three", false);
    SmartDashboard.putBoolean("four", false);
    SmartDashboard.putBoolean("five", false);
    SmartDashboard.putBoolean("six", false);
    SmartDashboard.putBoolean("seven", false);
    SmartDashboard.putBoolean("eight", false);
    SmartDashboard.putBoolean("nine", false);
  }

  public void gridderUpdater(double target) {
    if (target == 0) {
      dashBoardGridder();
      SmartDashboard.putBoolean("zero", true);
    } else if (target == 1) {
      dashBoardGridder();
      SmartDashboard.putBoolean("one", true);
    } else if (target == 2) {
      dashBoardGridder();
      SmartDashboard.putBoolean("two", true);
    } else if (target == 3) {
      dashBoardGridder();
      SmartDashboard.putBoolean("three", true);
    } else if (target == 4) {
      dashBoardGridder();
      SmartDashboard.putBoolean("four", true);
    } else if (target == 5) {
      dashBoardGridder();
      SmartDashboard.putBoolean("five", true);
    } else if (target == 6) {
      dashBoardGridder();
      SmartDashboard.putBoolean("six", true);
    } else if (target == 7) {
      dashBoardGridder();
      SmartDashboard.putBoolean("seven", true);
    } else if (target == 8) {
      dashBoardGridder();
      SmartDashboard.putBoolean("eight", true);
    } else if (target == 9) {
      dashBoardGridder();
      SmartDashboard.putBoolean("nine", true);
    }
  }
}
