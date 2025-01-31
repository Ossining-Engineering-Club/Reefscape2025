package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;
import frc.robot.subsystems.groundintakeroller.GroundIntakeRoller;

public class ReleaseAlgae extends SequentialCommandGroup {
    public ReleaseAlgae(GroundIntakePivot groundIntakePivot, CoralPivot coralPivot, Elevator elevator, AlgaeClaw algaeClaw) {
        addCommands(
            new GoToProcessingPosition(coralPivot, groundIntakePivot, elevator),
            Commands.runOnce(() -> algaeClaw.reverseMotor()),
            Commands.waitUntil(() -> !algaeClaw.hasAlgae()),
            new WaitCommand(AlgaeClawConstants.releaseDelaySeconds),
            Commands.runOnce(() -> algaeClaw.stopMotor())
        );
    }
}
