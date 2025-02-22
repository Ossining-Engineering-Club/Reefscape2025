package frc.robot.commands.autoteleop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.AlignmentConfig;
import frc.robot.commands.gamepiecemanipulation.GoToProcessingPosition;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoProcessAlgae extends SequentialCommandGroup {
  public AutoProcessAlgae(
      AlignmentConfig config, Pivot pivot, Elevator elevator, AlgaeClaw algaeClaw)
      throws FileVersionException, IOException, ParseException {
    PathPlannerPath path = PathPlannerPath.fromPathFile(config.pathName());
    Command pathFindingCommand =
        AutoBuilder.pathfindThenFollowPath(path, AutoTeleopConstants.processorAlignmentConstraints);

    addCommands(
        new ParallelCommandGroup(new GoToProcessingPosition(pivot, elevator), pathFindingCommand),
        algaeClaw.release());
  }
}
