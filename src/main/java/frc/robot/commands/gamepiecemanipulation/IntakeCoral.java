package frc.robot.commands.gamepiecemanipulation;

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

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(
      CoralPivot coralPivot, GroundIntakePivot groundIntakePivot, Elevator elevator) {
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ElevatorGoToHeight(elevator, ElevatorConstants.coralIntakeHeight),
                new CoralPivotGoToAngle(coralPivot, CoralPivotConstants.intakeAngle)),
            new GroundIntakePivotGoToAngle(
                groundIntakePivot, GroundIntakePivotConstants.stowAngle)));
  }
}
