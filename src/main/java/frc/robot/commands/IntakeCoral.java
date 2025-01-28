package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralpivot.CoralPivotGoToAngle;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotConstants;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(CoralPivot coralPivot, GroundIntakePivot groundIntakePivot) {
    addCommands(
        new ParallelCommandGroup(
            new CoralPivotGoToAngle(coralPivot, CoralPivotConstants.intakeAngle),
            new GroundIntakePivotGoToAngle(
                groundIntakePivot, GroundIntakePivotConstants.avoidanceAngle)),
        new GroundIntakePivotGoToAngle(groundIntakePivot, GroundIntakePivotConstants.stowAngle));
  }
}
