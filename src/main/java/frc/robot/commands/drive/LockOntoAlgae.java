package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import java.util.function.DoubleSupplier;

public class LockOntoAlgae extends Command {
    private final Drive drive;
    private final ObjectDetector algaeDetector;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;

    private final PIDController pid = new PIDController(1.0, 0, 0);

    public LockOntoAlgae(
            Drive drive,
            ObjectDetector algaeDetector,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        this.drive = drive;
        this.algaeDetector = algaeDetector;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void execute() {
        if (algaeDetector.recentlyHadTarget()) {
            DriveCommands.joystickDrive(
                            drive, xSupplier, ySupplier, () -> pid.calculate(algaeDetector.getYaw(), 0.0))
                    .schedule();
        } else {
            DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier).schedule();
        }
    }
}
