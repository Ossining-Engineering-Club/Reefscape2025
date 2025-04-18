package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class GoToNetAlgaePosition extends SequentialCommandGroup {
    public GoToNetAlgaePosition(Pivot pivot, Elevator elevator) {
        addCommands(
                new ParallelCommandGroup(
                                new ElevatorGoToHeight(elevator, ElevatorConstants.netHeight),
                                new PivotGoToAngle(pivot, Units.degreesToRadians(-90)))
                        .raceWith(new WaitCommand(3.5)),
                new PivotGoToAngle(pivot, PivotConstants.netAngle));
    }
}
