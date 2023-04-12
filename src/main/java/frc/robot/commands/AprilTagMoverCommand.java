package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.camera.CameraData;
import frc.robot.subsystems.camera.CameraData.TargetData;
import frc.robot.subsystems.camera.CameraInterface.PipelineType;

public class AprilTagMoverCommand extends CommandBase {
    private final SwerveSubsystem driveSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private boolean findReflectiveTarget = false;
    private boolean reachedTarget = false;
    Joystick joystick;
    Timer sustainedReach;

    public AprilTagMoverCommand(Joystick joystick, SwerveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.joystick = joystick;
        addRequirements(driveSubsystem);
        addRequirements(cameraSubsystem);
        cameraSubsystem.setPipeline(PipelineType.APRIL_TAG);
        sustainedReach = new Timer();
    }

    @Override
    public void initialize() {
        reachedTarget = false;
        sustainedReach.stop();
        sustainedReach.reset();
        Telemetry.logData("initialized", true, AprilTagMoverCommand.class);
    }

    @Override
    public void execute() {
        driveCloserToTarget_NoRetro();
    }

    @Override
    public boolean isFinished() {
        if (reachedTarget || (joystick != null && joystick.getRawButton(Constants.OIConstants.kButtonX) == false) ? true
                : false) {
            System.out.println("AprilTagMoveCommand: finished");
            Telemetry.logData("isFinished", true, AprilTagMoverCommand.class);
            return true;
        } else {
            return false;
        }
    }

    private void driveCloserToTarget_NoRetro() {

        CameraData camResult = cameraSubsystem.getTarget();
        if (camResult.pipelineType == PipelineType.APRIL_TAG) {
            double translation = 0.0;
            double yAxisTranslation = 0.0;
            Telemetry.logData("Has Targets", camResult.getTargets().get(0), AprilTagMoverCommand.class);
            if (camResult.hasTargets()) {
                TargetData target = camResult.getTargets().get(0);
                // if (target.targetDistance > 1)
                // translation = -0.6;
                // else if (target.targetDistance < .45)
                if (target.targetDistance > .88) {
                    translation = 0.6;
                    sustainedReach.stop();
                    sustainedReach.reset();
                } else {
                    translation = 0.0;
                    sustainedReach.start();
                    if (sustainedReach.hasElapsed(0.3))
                        reachedTarget = true;
                }
                double x = camResult.getTargets().get(0).targetXAngle;
                if (x > 2.0)
                    yAxisTranslation = -0.5;
                else if (x < -2.0)
                    yAxisTranslation = 0.5;
                else
                    yAxisTranslation = 0.5;

                Telemetry.logData("VX", translation, AprilTagMoverCommand.class);
                Telemetry.logData("VY", yAxisTranslation, AprilTagMoverCommand.class);
                Telemetry.logData("#Targets = ", camResult.getTargets().size(), AprilTagMoverCommand.class);
                Telemetry.logData("Distance Meters = ", camResult.getTargets().get(0).targetDistance,
                        AprilTagMoverCommand.class);
                Telemetry.logData("Angle Offset", x, AprilTagMoverCommand.class);
            }

            driveSubsystem.move(translation, yAxisTranslation, 0);
        }
    }

    /**
     * Calculate the left and right drive power based on distance and angle.
     * 
     * @param distance distance
     * @param angle    angle
     * @return the power for the left and right drive
     */
    protected static Pair<Double, Double> getDiffDriveValues(double distance, double angle) {
        final double maxSpeed = 0.7;
        double speed = maxSpeed;
        if (distance > 4)
            speed = maxSpeed * 0.7;
        else if (distance > 2)
            speed = maxSpeed;
        else if (distance > 1.3)
            speed = maxSpeed * 0.8;
        else if (distance > 1)
            speed = maxSpeed * 0.5;
        else if (distance < 0.7)
            speed = -maxSpeed * 0.5;
        else
            speed = 0;

        double turnRatio = 0;
        if (Math.abs(angle) > 3) {
            turnRatio = 0.7;
        } else if (Math.abs(angle) > 7) {
            turnRatio = 0.9;
        }
        if (angle < 0)
            turnRatio *= -1;
        if (speed < 0)
            turnRatio *= -1;
        double rightDriveSpeed = speed + (speed * turnRatio);
        double leftDriveSpeed = speed - (speed * turnRatio);
        return new Pair<Double, Double>(leftDriveSpeed, rightDriveSpeed);
    }

    protected static ChassisSpeeds getHolonomicDriveValues(double distance, double angle) {
        double x = 0;
        double y = 0;
        double rot = 0;

        if (distance > 1) {
            x = -0.5;
        } else if (distance < 0.45) {
            x = 0.5;
        } else {
            x = 0;
        }

        if (Math.abs(angle) > 2) {
            rot = Math.copySign(0.5, angle);
        } else {
            rot = 0;
        }

        return new ChassisSpeeds(x, y, rot);
    }

}
