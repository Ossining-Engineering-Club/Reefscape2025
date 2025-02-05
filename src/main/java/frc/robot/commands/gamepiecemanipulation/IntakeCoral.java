package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coralpivot.CoralPivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolderConstants;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(
      CoralPivot coralPivot,
      GroundIntakePivot groundIntakePivot,
      Elevator elevator,
      CoralHolder coralHolder) {
    addCommands(
        Commands.runOnce(() -> coralHolder.forward(), coralHolder),
        new ParallelDeadlineGroup(
            Commands.waitUntil(() -> coralHolder.hasCoral()),
            new SequentialCommandGroup(
                new ElevatorGoToHeight(elevator, ElevatorConstants.coralIntakeHeight),
                new CoralPivotGoToAngle(coralPivot, CoralPivotConstants.intakeAngle)),
            new GroundIntakePivotGoToAngle(
                groundIntakePivot, GroundIntakePivotConstants.stowAngle)),
        new WaitCommand(CoralHolderConstants.intakeDelaySeconds),
        Commands.runOnce(() -> coralHolder.stop(), coralHolder));
  }
}
