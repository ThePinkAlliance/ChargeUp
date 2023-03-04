// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import com.ThePinkAlliance.core.simulation.ctre.CtrePhysicsSim;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.motion.Profile;

public class PivotToDegreeProfiled extends CommandBase {
  private Profile motionProfile;

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
   * Since the CANTalon.set() routine is mode specific, deduce what we want
   * the set value to be and let the calling module apply it whenever we
   * decide to switch to MP mode.
   */
  private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;

  /**
   * Just a state timeout to make sure we don't get stuck anywhere. Each loop
   * is about 20ms.
   */
  private static final int kNumLoopsTimeout = 10;

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
  public PivotToDegreeProfiled(Profile motionProfile, ArmSubsystem armSubsystem) {
    this.motionProfile = motionProfile;
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

    _pos = _talon.getSelectedSensorPosition();
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

        double currentMotorRotations = (_pos + 1) / 2048;
        double targetMotorRotations = motionProfile.getDesiredRotations();

        if (currentMotorRotations <= targetMotorRotations + 2 && currentMotorRotations >= targetMotorRotations - 2) {
          _state = 2;
        } else {
          prepTalon(motionProfile);
          _state = 1;
        }

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
        _talon.startMotionProfile(motionProfile.toTrajectory(), 5, ControlMode.MotionProfile);
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

  private void prepTalon(Profile profile) {
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
    _talon.configMotionProfileTrajectoryPeriod(0, profile.getTimeoutMs());
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