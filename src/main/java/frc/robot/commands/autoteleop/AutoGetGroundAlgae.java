package frc.robot.commands.autoteleop;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LockOntoAlgae;
import frc.robot.commands.gamepiecemanipulation.GoToGroundAlgaePosition;
import frc.robot.commands.gamepiecemanipulation.IntakeGroundAlgae;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class AutoGetGroundAlgae extends SequentialCommandGroup {
    public AutoGetGroundAlgae(
            Elevator elevator,
            Pivot pivot,
            AlgaeClaw algaeClaw,
            Drive drive,
            ObjectDetector algaeDetector,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        addCommands(
                new GoToGroundAlgaePosition(elevator, pivot),
                new ParallelDeadlineGroup(
                        new IntakeGroundAlgae(elevator, pivot, algaeClaw),
                        new LockOntoAlgae(drive, algaeDetector, xSupplier, ySupplier, omegaSupplier)));
    }
}
