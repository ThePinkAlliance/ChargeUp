package frc.robot.network;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GridSubscriber {
    // the publisher is an instance variable so its lifetime matches that of the class
  DoubleSubscriber dblSub;

  public GridSubscriber(DoubleTopic dblTopic) {
    // subscribe options may be specified using PubSubOption
    dblSub = dblTopic.subscribe(99.0, PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
  }

  public void periodic() {
    // simple get of most recent value; if no value has been published,
    // returns the default value passed to the subscribe() function
    double val = dblSub.get();
    SmartDashboard.putNumber("GridPublisher CUI", val);
    gridderUpdater();
  }

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop publishing
  public void close() {
    // stop publishing
    dblSub.close();
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

  public void gridderUpdater() {
    double val = dblSub.get();

    if (val == 0) {
        dashBoardGridder();
        SmartDashboard.putBoolean("zero", true);
    }
    if (val == 1) {
        dashBoardGridder();
        SmartDashboard.putBoolean("one", true);
    }
    if (val == 2) {
        dashBoardGridder();
        SmartDashboard.putBoolean("two", true);
    }
    if (val == 3) {
        dashBoardGridder();
        SmartDashboard.putBoolean("three", true);
    }
    if (val == 4) {
        dashBoardGridder();
        SmartDashboard.putBoolean("four", true);
    }
    if (val == 5) {
        dashBoardGridder();
        SmartDashboard.putBoolean("five", true);
    }
    if (val == 6) {
        dashBoardGridder();
        SmartDashboard.putBoolean("six", true);
    }
    if (val == 7) {
        dashBoardGridder();
        SmartDashboard.putBoolean("seven", true);
    }
    if (val == 8) {
        dashBoardGridder();
        SmartDashboard.putBoolean("eight", true);
    }
    if (val == 9) {
        dashBoardGridder();
        SmartDashboard.putBoolean("nine", true);
    }

        
  }
}
