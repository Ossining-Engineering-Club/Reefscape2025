package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    Commands.waitUntil(() -> coralHolder.hasCoral()),
                    Commands.runOnce(() -> coralHolder.forward(), coralHolder),
                    new ElevatorGoToHeight(elevator, ElevatorConstants.intakeCoralHeight),
                    new PivotGoToAngle(pivot, PivotConstants.intakeCoralAngle)),
                new WaitCommand(CoralHolderConstants.intakeDelaySeconds),
                Commands.runOnce(() -> coralHolder.stop(), coralHolder)),
            Commands.runOnce(() -> {}),
            () -> !coralHolder.hasCoral()));
  }
}
