package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralpivot.CoralPivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;

public class GoToProcessorPosition extends SequentialCommandGroup {
  public GoToProcessorPosition(
      CoralPivot coralPivot, GroundIntakePivot groundIntakePivot, Elevator elevator) {
    addCommands(
        new ParallelCommandGroup(
            new ConditionalCommand(
                new GroundIntakePivotGoToAngle(
                    groundIntakePivot, GroundIntakePivotConstants.avoidanceAngle),
                Commands.runOnce(() -> {}),
                () ->
                    elevator.getHeight()
                        > ElevatorConstants.processorHeight + Units.inchesToMeters(1.0)),
            new ElevatorGoToHeight(elevator, ElevatorConstants.processorHeight),
            new CoralPivotGoToAngle(coralPivot, CoralPivotConstants.stowAngle)),
        new GroundIntakePivotGoToAngle(groundIntakePivot, GroundIntakePivotConstants.stowAngle));
  }
}
