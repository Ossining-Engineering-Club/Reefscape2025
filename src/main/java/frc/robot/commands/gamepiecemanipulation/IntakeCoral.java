package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolderConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(Pivot pivot, Elevator elevator, CoralHolder coralHolder) {
    addCommands(
        Commands.runOnce(() -> coralHolder.forward(), coralHolder),
        new ParallelDeadlineGroup(
            Commands.waitUntil(() -> coralHolder.hasCoral()),
            new SequentialCommandGroup(
                new ElevatorGoToHeight(elevator, ElevatorConstants.coralIntakeHeight),
                new PivotGoToAngle(pivot, PivotConstants.intakeAngle))),
        new WaitCommand(CoralHolderConstants.intakeDelaySeconds),
        Commands.runOnce(() -> coralHolder.stop(), coralHolder));
  }
}
