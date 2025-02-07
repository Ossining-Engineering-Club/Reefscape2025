package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.elevator.Elevator;

public class ReleaseAlgae extends SequentialCommandGroup {
  public ReleaseAlgae(
      Pivot pivot,
      Elevator elevator,
      AlgaeClaw algaeClaw) {
    addCommands(
        new GoToProcessingPosition(pivot, groundIntakePivot, elevator),
        Commands.runOnce(() -> algaeClaw.reverseMotor()),
        Commands.waitUntil(() -> !algaeClaw.hasAlgae()),
        new WaitCommand(AlgaeClawConstants.releaseDelaySeconds),
        Commands.runOnce(() -> algaeClaw.stopMotor()));
  }
}
