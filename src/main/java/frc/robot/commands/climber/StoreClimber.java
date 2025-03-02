package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.pivot.Pivot;

public class StoreClimber extends SequentialCommandGroup {
    public StoreClimber(Climber climber, Pivot pivot) {
        addCommands(new PivotGoToAngle(pivot, 0), climber.store());
    }
}
