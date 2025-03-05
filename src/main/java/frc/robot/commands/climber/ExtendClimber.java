package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

public class ExtendClimber extends SequentialCommandGroup {
    public ExtendClimber(Climber climber, Pivot pivot, Elevator elevator) {
        addCommands(
                new ParallelCommandGroup(
                        new PivotGoToAngle(pivot, 0), new ElevatorGoToHeight(elevator, 0)),
                climber.extend());
    }
}
