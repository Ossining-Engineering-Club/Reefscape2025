package frc.robot.commands.autoteleop;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.ReefAlgaeAlignmentConfig;
import frc.robot.commands.gamepiecemanipulation.IntakeReefAlgae;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;

public class AutoGetReefAlgae extends SequentialCommandGroup {
    public AutoGetReefAlgae(ReefAlgaeAlignmentConfig config, CoralPivot coralPivot, GroundIntakePivot groundIntakePivot, Elevator elevator) throws FileVersionException, IOException, ParseException {
        PathPlannerPath path = PathPlannerPath.fromPathFile(config.pathName());
        Command pathFindingCommand =
            AutoBuilder.pathfindThenFollowPath(path, AutoTeleopConstants.reefAlgaeAlignmentConstraints);
        
        double height = switch (config.pathName()) {
            case "CD", "GH", "KL" -> ElevatorConstants.lowerAlgaeHeight;
            case "AB", "EF", "IJ" -> ElevatorConstants.upperAlgaeHeight;
            default -> 0.0;
        };

        addCommands(
            new ParallelCommandGroup(
                pathFindingCommand,
                new IntakeReefAlgae(height, coralPivot, groundIntakePivot, elevator)
            )
        );
    }
}
