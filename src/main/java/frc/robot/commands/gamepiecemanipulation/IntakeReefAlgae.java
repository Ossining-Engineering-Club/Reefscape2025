package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class IntakeReefAlgae extends SequentialCommandGroup {
  public IntakeReefAlgae(double height, Pivot pivot, Elevator elevator, AlgaeClaw algaeClaw) {
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    Commands.waitUntil(() -> algaeClaw.hasAlgae()),
                    Commands.runOnce(() -> algaeClaw.startMotor(), algaeClaw),
                    new ElevatorGoToHeight(elevator, height),
                    new PivotGoToAngle(pivot, PivotConstants.intakeReefAlgaeAngle)),
                new WaitCommand(AlgaeClawConstants.intakeDelaySeconds),
                Commands.runOnce(() -> algaeClaw.stopMotor(), algaeClaw)),
            Commands.runOnce(() -> {}),
            () -> !algaeClaw.hasAlgae()));
  }
}
