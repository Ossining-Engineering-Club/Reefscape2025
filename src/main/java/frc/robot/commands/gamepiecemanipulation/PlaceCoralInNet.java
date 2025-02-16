package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class PlaceCoralInNet extends SequentialCommandGroup {
  public PlaceCoralInNet(Pivot pivot, Elevator elevator, AlgaeClaw algaeClaw) {
    addCommands(
        new ParallelCommandGroup(
            new ElevatorGoToHeight(elevator, ElevatorConstants.netHeight),
            new PivotGoToAngle(pivot, Units.degreesToRadians(-90))),
        new PivotGoToAngle(pivot, PivotConstants.netAngle),
        new ReleaseAlgae(algaeClaw));
  }
}
