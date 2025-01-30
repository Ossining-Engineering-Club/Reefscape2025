package frc.robot.commands.autoteleop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.AlignmentConfig;
import frc.robot.AutoTeleopConstants.Level;
import frc.robot.commands.gamepiecemanipulation.GoToPlacingCoralPosition;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoPlaceCoral extends SequentialCommandGroup {
  public AutoPlaceCoral(
      AlignmentConfig config,
      Level level,
      CoralPivot coralPivot,
      GroundIntakePivot groundIntakePivot,
      Elevator elevator)
      throws FileVersionException, IOException, ParseException {
    PathPlannerPath path = PathPlannerPath.fromPathFile(config.pathName());
    Command pathFindingCommand =
        AutoBuilder.pathfindThenFollowPath(path, AutoTeleopConstants.reefCoralAlignmentConstraints);

    double height =
        switch (level) {
          case L1 -> ElevatorConstants.l1Height;
          case L2 -> ElevatorConstants.l2Height;
          case L3 -> ElevatorConstants.l3Height;
          case L4 -> ElevatorConstants.l4Height;
          default -> 0.0;
        };

    addCommands(
        new ParallelCommandGroup(
            pathFindingCommand,
            new GoToPlacingCoralPosition(height, coralPivot, groundIntakePivot, elevator)));
  }
}
