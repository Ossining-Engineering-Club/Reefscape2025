package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coralpivot.CoralPivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;

public class IntakeGroundAlgae extends SequentialCommandGroup {
  public IntakeGroundAlgae(
      GroundIntakePivot groundIntakePivot, CoralPivot coralPivot, Elevator elevator) {
    addCommands(
        new ParallelCommandGroup(
            new GroundIntakePivotGoToAngle(
                groundIntakePivot, GroundIntakePivotConstants.extendAngle),
            new CoralPivotGoToAngle(coralPivot, CoralPivotConstants.stowAngle),
            new ElevatorGoToHeight(elevator, ElevatorConstants.groundAlgaeHeight)),
        new WaitCommand(3.0),
        new GroundIntakePivotGoToAngle(groundIntakePivot, GroundIntakePivotConstants.stowAngle));
  }
}
