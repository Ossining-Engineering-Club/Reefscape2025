package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolderConstants;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;

public class ReleaseCoral extends SequentialCommandGroup {
    public ReleaseCoral(CoralHolder coralHolder) {
        addCommands(
            Commands.runOnce(() -> coralHolder.forward(), coralHolder),
            Commands.waitUntil(() -> !coralHolder.hasCoral()),
            new WaitCommand(CoralHolderConstants.releaseDelaySeconds),
            Commands.runOnce(() -> coralHolder.stop(), coralHolder)
        );
    }
}
