package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoTeleopConstants.Level;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class GoToPlacingCoralPosition extends SequentialCommandGroup {
  public GoToPlacingCoralPosition(double height, Pivot pivot, Elevator elevator) {
    addCommands(
        new ParallelCommandGroup(
            new PivotGoToAngle(pivot, PivotConstants.placeCoralAngle)),
            new ElevatorGoToHeight(elevator, height));
  }
}
