package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoTeleopConstants.Level;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.pivot.pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;

public class GoToPlacingCoralPosition extends SequentialCommandGroup {
  public GoToPlacingCoralPosition(
      double height,
      Level level,
      pivot pivot,
      GroundIntakePivot groundIntakePivot,
      Elevator elevator) {
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(1),
                new PivotGoToAngle(
                    pivot,
                    (level == Level.L4)
                        ? PivotConstants.placeAngleL4
                        : PivotConstants.placeAngle)),
            new ElevatorGoToHeight(elevator, height),
            new GroundIntakePivotGoToAngle(
                groundIntakePivot, GroundIntakePivotConstants.stowAngle)));
  }
}
