package frc.robot.commands.autoteleop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.AlignmentConfig;
import frc.robot.commands.gamepiecemanipulation.GoToProcessingPosition;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.elevator.Elevator;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoProcessAlgae extends SequentialCommandGroup {
  public AutoProcessAlgae(
      AlignmentConfig config,
      Pivot pivot,
      Elevator elevator)
      throws FileVersionException, IOException, ParseException {

    Command pathFindingCommand1 =
        AutoBuilder.pathfindToPose(
            AutoTeleopConstants.clearProcessorPose,
            AutoTeleopConstants.processorAlignmentConstraints,
            0.0);
    PathPlannerPath path = PathPlannerPath.fromPathFile(config.pathName());
    Command pathFindingCommand2 =
        AutoBuilder.pathfindThenFollowPath(path, AutoTeleopConstants.processorAlignmentConstraints);

    addCommands(
        new ParallelCommandGroup(
            pathFindingCommand1,
            new SequentialCommandGroup(
                new WaitCommand(1.0),
                new GoToProcessingPosition(pivot, elevator))),
        pathFindingCommand2);
  }
}
