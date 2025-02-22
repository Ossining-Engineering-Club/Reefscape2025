package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class IntakeGroundAlgae extends SequentialCommandGroup {
  public IntakeGroundAlgae(Elevator elevator, Pivot pivot, AlgaeClaw algaeClaw) {
    addCommands(
        new ParallelDeadlineGroup(
            algaeClaw.intake(),
            new ElevatorGoToHeight(elevator, ElevatorConstants.groundAlgaeHeight),
            new PivotGoToAngle(pivot, PivotConstants.groundAlgaeAngle)));
  }
}
