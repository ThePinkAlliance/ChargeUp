// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import com.ThePinkAlliance.core.simulation.ctre.CtrePhysicsSim;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class PivotToDegreeProfiled extends CommandBase {
  public static final int kNumPoints = 358;

  // Position (rotations) Velocity (RPM) Duration (ms)
  public static double[][] Points = new double[][] {
      { 0, 0, 40 },
      { 0.008, 24, 40 },
      { 0.036, 60, 40 },
      { 0.092, 108, 40 },
      { 0.184, 168, 40 },
      { 0.32, 240, 40 },
      { 0.496, 288, 40 },
      { 0.7, 324, 40 },
      { 0.924, 348, 40 },
      { 1.16, 360, 40 },
      { 1.4, 360, 40 },
      { 1.64, 360, 40 },
      { 1.88, 360, 40 },
      { 2.12, 360, 40 },
      { 2.36, 360, 40 },
      { 2.6, 360, 40 },
      { 2.84, 360, 40 },
      { 3.08, 360, 40 },
      { 3.32, 360, 40 },
      { 3.56, 360, 40 },
      { 3.8, 360, 40 },
      { 4.04, 360, 40 },
      { 4.28, 360, 40 },
      { 4.52, 360, 40 },
      { 4.76, 360, 40 },
      { 5, 360, 40 },
      { 5.24, 360, 40 },
      { 5.48, 360, 40 },
      { 5.72, 360, 40 },
      { 5.96, 360, 40 },
      { 6.2, 360, 40 },
      { 6.44, 360, 40 },
      { 6.68, 360, 40 },
      { 6.92, 360, 40 },
      { 7.16, 360, 40 },
      { 7.4, 360, 40 },
      { 7.64000000000001, 360, 40 },
      { 7.88000000000001, 360, 40 },
      { 8.12, 360, 40 },
      { 8.36, 360, 40 },
      { 8.6, 360, 40 },
      { 8.84000000000001, 360, 40 },
      { 9.08000000000001, 360, 40 },
      { 9.32000000000001, 360, 40 },
      { 9.56000000000001, 360, 40 },
      { 9.80000000000001, 360, 40 },
      { 10.04, 360, 40 },
      { 10.28, 360, 40 },
      { 10.52, 360, 40 },
      { 10.76, 360, 40 },
      { 11, 360, 40 },
      { 11.24, 360, 40 },
      { 11.48, 360, 40 },
      { 11.72, 360, 40 },
      { 11.96, 360, 40 },
      { 12.2, 360, 40 },
      { 12.44, 360, 40 },
      { 12.68, 360, 40 },
      { 12.92, 360, 40 },
      { 13.16, 360, 40 },
      { 13.4, 360, 40 },
      { 13.64, 360, 40 },
      { 13.88, 360, 40 },
      { 14.12, 360, 40 },
      { 14.36, 360, 40 },
      { 14.6, 360, 40 },
      { 14.84, 360, 40 },
      { 15.08, 360, 40 },
      { 15.32, 360, 40 },
      { 15.56, 360, 40 },
      { 15.8, 360, 40 },
      { 16.04, 360, 40 },
      { 16.28, 360, 40 },
      { 16.52, 360, 40 },
      { 16.76, 360, 40 },
      { 17, 360, 40 },
      { 17.24, 360, 40 },
      { 17.48, 360, 40 },
      { 17.72, 360, 40 },
      { 17.96, 360, 40 },
      { 18.2, 360, 40 },
      { 18.44, 360, 40 },
      { 18.68, 360, 40 },
      { 18.92, 360, 40 },
      { 19.16, 360, 40 },
      { 19.4, 360, 40 },
      { 19.64, 360, 40 },
      { 19.88, 360, 40 },
      { 20.12, 360, 40 },
      { 20.36, 360, 40 },
      { 20.6, 360, 40 },
      { 20.84, 360, 40 },
      { 21.08, 360, 40 },
      { 21.32, 360, 40 },
      { 21.56, 360, 40 },
      { 21.8, 360, 40 },
      { 22.04, 360, 40 },
      { 22.28, 360, 40 },
      { 22.52, 360, 40 },
      { 22.76, 360, 40 },
      { 23, 360, 40 },
      { 23.24, 360, 40 },
      { 23.48, 360, 40 },
      { 23.72, 360, 40 },
      { 23.96, 360, 40 },
      { 24.2, 360, 40 },
      { 24.44, 360, 40 },
      { 24.68, 360, 40 },
      { 24.92, 360, 40 },
      { 25.16, 360, 40 },
      { 25.3999999999999, 360, 40 },
      { 25.6399999999999, 360, 40 },
      { 25.8799999999999, 360, 40 },
      { 26.1199999999999, 360, 40 },
      { 26.3599999999999, 360, 40 },
      { 26.5999999999999, 360, 40 },
      { 26.8399999999999, 360, 40 },
      { 27.0799999999999, 360, 40 },
      { 27.3199999999999, 360, 40 },
      { 27.5599999999999, 360, 40 },
      { 27.7999999999999, 360, 40 },
      { 28.0399999999999, 360, 40 },
      { 28.2799999999999, 360, 40 },
      { 28.5199999999999, 360, 40 },
      { 28.7599999999999, 360, 40 },
      { 28.9999999999999, 360, 40 },
      { 29.2399999999999, 360, 40 },
      { 29.4799999999999, 360, 40 },
      { 29.7199999999999, 360, 40 },
      { 29.9599999999999, 360, 40 },
      { 30.1999999999999, 360, 40 },
      { 30.4399999999999, 360, 40 },
      { 30.6799999999999, 360, 40 },
      { 30.9199999999999, 360, 40 },
      { 31.1599999999999, 360, 40 },
      { 31.3999999999999, 360, 40 },
      { 31.6399999999999, 360, 40 },
      { 31.8799999999999, 360, 40 },
      { 32.1199999999999, 360, 40 },
      { 32.3599999999999, 360, 40 },
      { 32.5999999999999, 360, 40 },
      { 32.8399999999999, 360, 40 },
      { 33.0799999999999, 360, 40 },
      { 33.3199999999999, 360, 40 },
      { 33.5599999999999, 360, 40 },
      { 33.7999999999999, 360, 40 },
      { 34.0399999999999, 360, 40 },
      { 34.2799999999999, 360, 40 },
      { 34.5199999999999, 360, 40 },
      { 34.7599999999999, 360, 40 },
      { 34.9999999999999, 360, 40 },
      { 35.2399999999999, 360, 40 },
      { 35.4799999999999, 360, 40 },
      { 35.7199999999999, 360, 40 },
      { 35.9599999999999, 360, 40 },
      { 36.1999999999999, 360, 40 },
      { 36.4399999999999, 360, 40 },
      { 36.6799999999999, 360, 40 },
      { 36.9199999999999, 360, 40 },
      { 37.1599999999999, 360, 40 },
      { 37.3999999999999, 360, 40 },
      { 37.64, 360, 40 },
      { 37.88, 360, 40 },
      { 38.12, 360, 40 },
      { 38.36, 360, 40 },
      { 38.6, 360, 40 },
      { 38.84, 360, 40 },
      { 39.08, 360, 40 },
      { 39.32, 360, 40 },
      { 39.56, 360, 40 },
      { 39.8, 360, 40 },
      { 40.04, 360, 40 },
      { 40.28, 360, 40 },
      { 40.52, 360, 40 },
      { 40.76, 360, 40 },
      { 41, 360, 40 },
      { 41.24, 360, 40 },
      { 41.48, 360, 40 },
      { 41.72, 360, 40 },
      { 41.96, 360, 40 },
      { 42.2, 360, 40 },
      { 42.44, 360, 40 },
      { 42.68, 360, 40 },
      { 42.92, 360, 40 },
      { 43.16, 360, 40 },
      { 43.4, 360, 40 },
      { 43.64, 360, 40 },
      { 43.88, 360, 40 },
      { 44.12, 360, 40 },
      { 44.36, 360, 40 },
      { 44.6, 360, 40 },
      { 44.84, 360, 40 },
      { 45.08, 360, 40 },
      { 45.32, 360, 40 },
      { 45.56, 360, 40 },
      { 45.8, 360, 40 },
      { 46.04, 360, 40 },
      { 46.28, 360, 40 },
      { 46.52, 360, 40 },
      { 46.76, 360, 40 },
      { 47, 360, 40 },
      { 47.24, 360, 40 },
      { 47.48, 360, 40 },
      { 47.72, 360, 40 },
      { 47.96, 360, 40 },
      { 48.2, 360, 40 },
      { 48.44, 360, 40 },
      { 48.68, 360, 40 },
      { 48.92, 360, 40 },
      { 49.16, 360, 40 },
      { 49.4, 360, 40 },
      { 49.6400000000001, 360, 40 },
      { 49.8800000000001, 360, 40 },
      { 50.1200000000001, 360, 40 },
      { 50.3600000000001, 360, 40 },
      { 50.6000000000001, 360, 40 },
      { 50.8400000000001, 360, 40 },
      { 51.0800000000001, 360, 40 },
      { 51.3200000000001, 360, 40 },
      { 51.5600000000001, 360, 40 },
      { 51.8000000000001, 360, 40 },
      { 52.0400000000001, 360, 40 },
      { 52.2800000000001, 360, 40 },
      { 52.5200000000001, 360, 40 },
      { 52.7600000000001, 360, 40 },
      { 53.0000000000001, 360, 40 },
      { 53.2400000000001, 360, 40 },
      { 53.4800000000001, 360, 40 },
      { 53.7200000000001, 360, 40 },
      { 53.9600000000001, 360, 40 },
      { 54.2000000000001, 360, 40 },
      { 54.4400000000001, 360, 40 },
      { 54.6800000000001, 360, 40 },
      { 54.9200000000001, 360, 40 },
      { 55.1600000000001, 360, 40 },
      { 55.4000000000001, 360, 40 },
      { 55.6400000000001, 360, 40 },
      { 55.8800000000001, 360, 40 },
      { 56.1200000000001, 360, 40 },
      { 56.3600000000001, 360, 40 },
      { 56.6000000000001, 360, 40 },
      { 56.8400000000001, 360, 40 },
      { 57.0800000000001, 360, 40 },
      { 57.3200000000001, 360, 40 },
      { 57.5600000000001, 360, 40 },
      { 57.8000000000001, 360, 40 },
      { 58.0400000000001, 360, 40 },
      { 58.2800000000001, 360, 40 },
      { 58.5200000000001, 360, 40 },
      { 58.7600000000001, 360, 40 },
      { 59.0000000000001, 360, 40 },
      { 59.2400000000001, 360, 40 },
      { 59.4800000000001, 360, 40 },
      { 59.7200000000001, 360, 40 },
      { 59.9600000000001, 360, 40 },
      { 60.2000000000001, 360, 40 },
      { 60.4400000000001, 360, 40 },
      { 60.6800000000001, 360, 40 },
      { 60.9200000000001, 360, 40 },
      { 61.1600000000001, 360, 40 },
      { 61.4000000000001, 360, 40 },
      { 61.6400000000001, 360, 40 },
      { 61.8800000000002, 360, 40 },
      { 62.1200000000002, 360, 40 },
      { 62.3600000000002, 360, 40 },
      { 62.6000000000002, 360, 40 },
      { 62.8400000000002, 360, 40 },
      { 63.0800000000002, 360, 40 },
      { 63.3200000000002, 360, 40 },
      { 63.5600000000002, 360, 40 },
      { 63.8000000000002, 360, 40 },
      { 64.0400000000002, 360, 40 },
      { 64.2800000000002, 360, 40 },
      { 64.5200000000002, 360, 40 },
      { 64.7600000000001, 360, 40 },
      { 65.0000000000001, 360, 40 },
      { 65.2400000000001, 360, 40 },
      { 65.4800000000001, 360, 40 },
      { 65.7200000000001, 360, 40 },
      { 65.9600000000001, 360, 40 },
      { 66.2000000000001, 360, 40 },
      { 66.4400000000001, 360, 40 },
      { 66.6800000000001, 360, 40 },
      { 66.9200000000001, 360, 40 },
      { 67.1600000000001, 360, 40 },
      { 67.4000000000001, 360, 40 },
      { 67.6400000000001, 360, 40 },
      { 67.8800000000001, 360, 40 },
      { 68.1200000000001, 360, 40 },
      { 68.3600000000001, 360, 40 },
      { 68.6000000000001, 360, 40 },
      { 68.8400000000001, 360, 40 },
      { 69.0800000000001, 360, 40 },
      { 69.3200000000001, 360, 40 },
      { 69.56, 360, 40 },
      { 69.8, 360, 40 },
      { 70.04, 360, 40 },
      { 70.28, 360, 40 },
      { 70.52, 360, 40 },
      { 70.76, 360, 40 },
      { 71, 360, 40 },
      { 71.24, 360, 40 },
      { 71.48, 360, 40 },
      { 71.72, 360, 40 },
      { 71.96, 360, 40 },
      { 72.2, 360, 40 },
      { 72.44, 360, 40 },
      { 72.68, 360, 40 },
      { 72.92, 360, 40 },
      { 73.16, 360, 40 },
      { 73.4, 360, 40 },
      { 73.64, 360, 40 },
      { 73.88, 360, 40 },
      { 74.1199999999999, 360, 40 },
      { 74.3599999999999, 360, 40 },
      { 74.5999999999999, 360, 40 },
      { 74.8399999999999, 360, 40 },
      { 75.0799999999999, 360, 40 },
      { 75.3199999999999, 360, 40 },
      { 75.5599999999999, 360, 40 },
      { 75.7999999999999, 360, 40 },
      { 76.0399999999999, 360, 40 },
      { 76.2799999999999, 360, 40 },
      { 76.5199999999999, 360, 40 },
      { 76.7599999999999, 360, 40 },
      { 76.9999999999999, 360, 40 },
      { 77.2399999999999, 360, 40 },
      { 77.4799999999999, 360, 40 },
      { 77.7199999999999, 360, 40 },
      { 77.9599999999999, 360, 40 },
      { 78.1999999999999, 360, 40 },
      { 78.4399999999999, 360, 40 },
      { 78.6799999999999, 360, 40 },
      { 78.9199999999998, 360, 40 },
      { 79.1599999999998, 360, 40 },
      { 79.3999999999998, 360, 40 },
      { 79.6399999999998, 360, 40 },
      { 79.8799999999998, 360, 40 },
      { 80.1199999999998, 360, 40 },
      { 80.3599999999998, 360, 40 },
      { 80.5999999999998, 360, 40 },
      { 80.8399999999998, 360, 40 },
      { 81.0799999999998, 360, 40 },
      { 81.3199999999998, 360, 40 },
      { 81.5599999999998, 360, 40 },
      { 81.7999999999998, 360, 40 },
      { 82.0399999999998, 360, 40 },
      { 82.2799999999998, 360, 40 },
      { 82.5119999999998, 336, 40 },
      { 82.7239999999998, 300, 40 },
      { 82.9079999999998, 252, 40 },
      { 83.0559999999998, 192, 40 },
      { 83.1599999999998, 120, 40 },
      { 83.2239999999998, 72, 40 },
      { 83.2599999999998, 36, 40 },
      { 83.2759999999998, 12, 40 },
      { 83.2799999999998, 3.33067E-15, 40 },
      { 83.2799999999998, 0, 40 } };

  /**
   * The status of the motion profile executer and buffer inside the Talon.
   * Instead of creating a new one every time we call getMotionProfileStatus,
   * keep one copy.
   */
  private MotionProfileStatus _status = new MotionProfileStatus();

  /** additional cache for holding the active trajectory point */
  double _pos = 0, _vel = 0, _heading = 0;

  /**
   * reference to the talon we plan on manipulating. We will not changeMode()
   * or call set(), just get motion profile status and make decisions based on
   * motion profile.
   */
  private TalonFX _talon;
  /**
   * State machine to make sure we let enough of the motion profile stream to
   * talon before we fire it.
   */
  private int _state = 0;
  /**
   * Any time you have a state machine that waits for external events, its a
   * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
   * down to '0' which will print an error message. Counting loops is not a
   * very accurate method of tracking timeout, but this is just conservative
   * timeout. Getting time-stamps would certainly work too, this is just
   * simple (no need to worry about timer overflows).
   */
  private int _loopTimeout = -1;
  /**
   * If start() gets called, this flag is set and in the control() we will
   * service it.
   */
  private boolean _bStart = false;

  /**
   * Since the CANTalon.set() routine is mode specific, deduce what we want
   * the set value to be and let the calling module apply it whenever we
   * decide to switch to MP mode.
   */
  private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
  /**
   * How many trajectory points do we wait for before firing the motion
   * profile.
   */
  private static final int kMinPointsInTalon = 5;
  /**
   * Just a state timeout to make sure we don't get stuck anywhere. Each loop
   * is about 20ms.
   */
  private static final int kNumLoopsTimeout = 10;

  BufferedTrajectoryPointStream pointStream = new BufferedTrajectoryPointStream();

  /**
   * Lets create a periodic task to funnel our trajectory points into our talon.
   * It doesn't need to be very accurate, just needs to keep pace with the motion
   * profiler executer. Now if you're trajectory points are slow, there is no need
   * to do this, just call _talon.processMotionProfileBuffer() in your teleop
   * loop.
   * Generally speaking you want to call it at least twice as fast as the duration
   * of your trajectory points. So if they are firing every 20ms, you should call
   * every 10ms.
   */
  class PeriodicRunnable implements java.lang.Runnable {
    public void run() {
      _talon.processMotionProfileBuffer();
    }
  }

  Notifier _notifer = new Notifier(new PeriodicRunnable());

  /** Creates a new PivotToDegreeProfiled. */
  public PivotToDegreeProfiled(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    /*
     * since our MP is 10ms per point, set the control frame rate and the
     * notifer to half that
     */
    _talon = armSubsystem.getPivotTalon();

    CtrePhysicsSim.getInstance().addTalonFX(_talon, 0.5, 5100);

    _talon.changeMotionControlFramePeriod(5);
    _notifer.startPeriodic(0.005);

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _talon.set(ControlMode.MotionProfile, _setValue.value);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get the motion profile status every loop */
    _talon.getMotionProfileStatus(_status);

    /*
     * we are in MP control mode. That means: starting Mps, checking Mp
     * progress, and possibly interrupting MPs if thats what you want to
     * do.
     */
    switch (_state) {
      case 0: /* wait for application to tell us to start an MP */
        _setValue = SetValueMotionProfile.Disable;
        startFilling(Points, kNumPoints);
        /*
         * MP is being sent to CAN bus, wait a small amount of time
         */
        _state = 1;
        _loopTimeout = kNumLoopsTimeout;
        break;
      case 1: /*
               * wait for MP to stream to Talon, really just the first few
               * points
               */
        /* do we have a minimum numberof points in Talon */
        /* start (once) the motion profile */
        _setValue = SetValueMotionProfile.Enable;
        /* MP will start once the control frame gets scheduled */
        _talon.startMotionProfile(pointStream, 5, ControlMode.MotionProfile);
        _state = 2;
        _loopTimeout = kNumLoopsTimeout;
        break;
      case 2: /* check the status of the MP */
        /*
         * if talon is reporting things are good, keep adding to our
         * timeout. Really this is so that you can unplug your talon in
         * the middle of an MP and react to it.
         */
        if (_status.isUnderrun == false) {
          _loopTimeout = kNumLoopsTimeout;
        }
        /*
         * If we are executing an MP and the MP finished, start loading
         * another. We will go into hold state so robot servo's
         * position.
         */
        if (_status.activePointValid && _status.isLast) {
          /*
           * because we set the last point's isLast to true, we will
           * get here when the MP is done
           */
          _setValue = SetValueMotionProfile.Hold;
          _state = 0;
          _loopTimeout = -1;
        }
        break;
    }

    /* Get the motion profile status every loop */
    _talon.getMotionProfileStatus(_status);
    _heading = 0;
    _pos = _talon.getActiveTrajectoryPosition();
    _vel = _talon.getActiveTrajectoryVelocity();

    /* printfs and/or logging */
    System.out.println(
        "[Profile]: State: " + _state + " Position: " + _pos + " Velocity: " + _vel + " Heading: " + _heading);
  }

  /**
   * Called to clear Motion profile buffer and reset state info during
   * disabled and when Talon is not in MP control mode.
   */
  public void reset() {
    /*
     * Let's clear the buffer just in case user decided to disable in the
     * middle of an MP, and now we have the second half of a profile just
     * sitting in memory.
     */
    _talon.clearMotionProfileTrajectories();
    /* When we do re-enter motionProfile control mode, stay disabled. */
    _setValue = SetValueMotionProfile.Disable;
    /* When we do start running our state machine start at the beginning. */
    _state = 0;
    _loopTimeout = -1;
    /*
     * If application wanted to start an MP before, ignore and wait for next
     * button press
     */
    _bStart = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * track time, this is rudimentary but that's okay, we just want to make
     * sure things never get stuck.
     */
    if (_loopTimeout < 0) {
      /* do nothing, timeout is disabled */
    } else {
      /* our timeout is nonzero */
      if (_loopTimeout == 0) {
        /*
         * something is wrong. Talon is not present, unplugged, breaker
         * tripped
         */
        System.out.println("[Profile]: Error loop timeout non-zero");
        return true;
      } else {
        --_loopTimeout;
      }
    }

    return _status.isLast;
  }

  private void startFilling(double[][] profile, int totalCnt) {

    /* create an empty point */
    TrajectoryPoint point = new TrajectoryPoint();

    /* did we get an underrun condition since last time we checked ? */
    if (_status.hasUnderrun) {
      /* better log it so we know about it */
      System.out.println("[Profile]: profile has underrun");
      /*
       * clear the error. This flag does not auto clear, this way
       * we never miss logging it.
       */
      _talon.clearMotionProfileHasUnderrun(0);
    }
    /*
     * just in case we are interrupting another MP and there is still buffer
     * points in memory, clear it.
     */
    _talon.clearMotionProfileTrajectories();

    /*
     * set the base trajectory period to zero, use the individual trajectory period
     * below
     */
    _talon.configMotionProfileTrajectoryPeriod(0, 30);

    /* This is fast since it's just into our TOP buffer */
    for (int i = 0; i < totalCnt; ++i) {
      double positionRot = profile[i][0];
      double velocityRPM = profile[i][1];
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
      if ((i + 1) == totalCnt)
        point.isLastPoint = true; /* set this to true on the last point */

      // _talon.pushMotionProfileTrajectory(point);
      pointStream.Write(point);
    }
  }

  /**
   * Called by application to signal Talon to start the buffered MP (when it's
   * able to).
   */
  void startMotionProfile() {
    _bStart = true;
  }

  /**
   * 
   * @return the output value to pass to Talon's set() routine. 0 for disable
   *         motion-profile output, 1 for enable motion-profile, 2 for hold
   *         current motion profile trajectory point.
   */
  SetValueMotionProfile getSetValue() {
    return _setValue;
  }
}