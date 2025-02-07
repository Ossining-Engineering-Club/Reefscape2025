package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.elevator.Elevator;



public class IntakeReefAlgae extends SequentialCommandGroup {
  public IntakeReefAlgae(
      double height,
      Pivot pivot,
      GroundIntakePivot groundIntakePivot,
      Elevator elevator,
      AlgaeClaw algaeClaw) {
    addCommands(
        Commands.runOnce(() -> algaeClaw.startMotor(), algaeClaw),
        new ParallelDeadlineGroup(
            Commands.waitUntil(() -> algaeClaw.hasAlgae()),
            new ElevatorGoToHeight(elevator, height),
            new GroundIntakePivotGoToAngle(
                groundIntakePivot, GroundIntakePivotConstants.stowAngle)),
        new WaitCommand(AlgaeClawConstants.intakeDelaySeconds),
        Commands.runOnce(() -> algaeClaw.stopMotor(), algaeClaw),
        new PivotGoToAngle(pivot, PivotConstants.reefAlgaeAngle));
  }
}
