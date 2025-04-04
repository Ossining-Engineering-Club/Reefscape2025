package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class DriveBackAlgae extends SequentialCommandGroup {
    public DriveBackAlgae(Drive drive) {
        addCommands(
                Commands.race(
                                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 1, 0))),
                                new WaitCommand(0.3))
                        .andThen(Commands.runOnce(() -> drive.stop())));
    }
}
