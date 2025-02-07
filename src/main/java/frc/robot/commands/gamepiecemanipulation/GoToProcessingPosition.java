package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class GoToProcessingPosition extends SequentialCommandGroup {
  public GoToProcessingPosition(
      Pivot pivot, Elevator elevator) {
    addCommands(
        new ParallelCommandGroup(
            new ElevatorGoToHeight(elevator, ElevatorConstants.processorHeight),
            new PivotGoToAngle(pivot, PivotConstants.stowAngle)));
  }
}
