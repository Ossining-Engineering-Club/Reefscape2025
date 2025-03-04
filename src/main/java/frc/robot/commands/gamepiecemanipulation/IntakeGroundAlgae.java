package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

public class IntakeGroundAlgae extends SequentialCommandGroup {
    public IntakeGroundAlgae(Elevator elevator, Pivot pivot, AlgaeClaw algaeClaw) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(new WaitCommand(1), algaeClaw.intakeGround()),
                        new GoToGroundAlgaePosition(elevator, pivot)));
    }
}
