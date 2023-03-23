package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup(SwerveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        addCommands(new AprilTagMoverCommand(null, driveSubsystem, cameraSubsystem)); 
    }
}
