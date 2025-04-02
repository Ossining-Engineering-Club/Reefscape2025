package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class IntakeCoralAuto extends SequentialCommandGroup {
    public IntakeCoralAuto(Pivot pivot, Elevator elevator, CoralHolder coralHolder) {
        addCommands(
                new ConditionalCommand(
                        new ParallelDeadlineGroup(
                                coralHolder.intake(),
                                new ElevatorGoToHeight(
                                        elevator, ElevatorConstants.intakeCoralAutoHeight),
                                new PivotGoToAngle(pivot, PivotConstants.intakeCoralAutoAngle)),
                        Commands.runOnce(() -> {}),
                        () -> !coralHolder.hasCoral()));
    }
}
