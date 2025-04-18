package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class DriveBackAfterNet extends SequentialCommandGroup {
    public DriveBackAfterNet(Drive drive) {
        addCommands(
                Commands.race(
                                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 1, 0))),
                                new WaitCommand(1.0))
                        .andThen(Commands.runOnce(() -> drive.stop())));
    }
}
