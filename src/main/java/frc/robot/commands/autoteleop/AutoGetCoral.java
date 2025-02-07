package frc.robot.commands.autoteleop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.AlignmentConfig;
import frc.robot.commands.gamepiecemanipulation.IntakeCoral;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.pivot.pivot;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoGetCoral extends SequentialCommandGroup {
  public AutoGetCoral(
      AlignmentConfig config,
      pivot pivot,
      GroundIntakePivot groundIntakePivot,
      Elevator elevator,
      CoralHolder coralHolder)
      throws FileVersionException, IOException, ParseException {
    PathPlannerPath path = PathPlannerPath.fromPathFile(config.pathName());
    Command pathFindingCommand =
        AutoBuilder.pathfindThenFollowPath(
            path, AutoTeleopConstants.coralStationAlignmentConstraints);

    addCommands(
        new ParallelCommandGroup(
            pathFindingCommand,
            new IntakeCoral(pivot, groundIntakePivot, elevator, coralHolder)));
  }
}
