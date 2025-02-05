package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coralpivot.CoralPivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;

public class IntakeReefAlgae extends SequentialCommandGroup {
  public IntakeReefAlgae(
      double height,
      CoralPivot coralPivot,
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
        new CoralPivotGoToAngle(coralPivot, CoralPivotConstants.reefAlgaeAngle));
  }
}
