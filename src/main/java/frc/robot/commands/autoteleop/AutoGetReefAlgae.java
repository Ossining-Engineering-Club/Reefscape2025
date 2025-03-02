package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.getTagIdOfPosition;
import static frc.robot.AutoTeleopConstants.reefAlgaeAlignmentConstraints;
import static frc.robot.AutoTeleopConstants.switchingToSpecializedRotationalTolerance;
import static frc.robot.AutoTeleopConstants.switchingToSpecializedTranslationalTolerance;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants.PositioningConfig;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldSimulationManager;
import frc.robot.FieldSimulationManager.Objective;
import frc.robot.commands.gamepiecemanipulation.IntakeReefAlgae;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AutoGetReefAlgae extends SequentialCommandGroup {
    public AutoGetReefAlgae(
            PositioningConfig config,
            Pivot pivot,
            Elevator elevator,
            AlgaeClaw algaeClaw,
            Drive drive,
            Vision vision)
            throws FileVersionException, IOException, ParseException {
        Pose2d targetPoseBlue =
                GoToPositionSpecialized.getTargetPose(
                                getTagIdOfPosition(config.position(), Alliance.Blue),
                                config.sidewaysOffset(),
                                config.depthOffset(),
                                false)
                        .get();
        Pose2d targetPoseRed =
                GoToPositionSpecialized.getTargetPose(
                                getTagIdOfPosition(config.position(), Alliance.Red),
                                config.sidewaysOffset(),
                                config.depthOffset(),
                                false)
                        .get();
        Command pathfindingCommandBlue =
                AutoBuilder.pathfindToPose(targetPoseBlue, reefAlgaeAlignmentConstraints, 0.0);
        Command pathfindingCommandRed =
                AutoBuilder.pathfindToPose(targetPoseRed, reefAlgaeAlignmentConstraints, 0.0);

        double height =
                switch (config.position()) {
                    case CD, GH, KL -> ElevatorConstants.lowerAlgaeHeight;
                    case AB, EF, IJ -> ElevatorConstants.upperAlgaeHeight;
                    default -> 0.0;
                };

        addCommands(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        Commands.runOnce(
                                                () ->
                                                        Logger.recordOutput(
                                                                "initial target pose",
                                                                targetPoseBlue)),
                                        pathfindingCommandBlue.until(
                                                () ->
                                                        Math.hypot(
                                                                                drive.getPose()
                                                                                                .getX()
                                                                                        - targetPoseBlue
                                                                                                .getX(),
                                                                                drive.getPose()
                                                                                                .getY()
                                                                                        - targetPoseBlue
                                                                                                .getY())
                                                                        <= switchingToSpecializedTranslationalTolerance
                                                                && Math.abs(
                                                                                drive.getRotation()
                                                                                                .getRadians()
                                                                                        - targetPoseBlue
                                                                                                .getRotation()
                                                                                                .getRadians())
                                                                        <= switchingToSpecializedRotationalTolerance
                                                                && vision.seesFocusTag()),
                                        new GoToPositionSpecialized(
                                                drive,
                                                vision,
                                                config.position(),
                                                config.sidewaysOffset(),
                                                config.depthOffset(),
                                                reefAlgaeAlignmentConstraints),
                                        new ConditionalCommand(
                                                new ConditionalCommand(
                                                        Commands.runOnce(
                                                                () ->
                                                                        FieldSimulationManager
                                                                                .setCurrentObjective(
                                                                                        Objective
                                                                                                .UPPER_REEF_ALGAE)),
                                                        Commands.runOnce(
                                                                () ->
                                                                        FieldSimulationManager
                                                                                .setCurrentObjective(
                                                                                        Objective
                                                                                                .LOWER_REEF_ALGAE)),
                                                        () ->
                                                                height
                                                                        == ElevatorConstants
                                                                                .upperAlgaeHeight),
                                                Commands.runOnce(() -> {}),
                                                () -> Constants.currentMode == Mode.SIM)),
                                new IntakeReefAlgae(height, pivot, elevator, algaeClaw)),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        Commands.runOnce(
                                                () ->
                                                        Logger.recordOutput(
                                                                "initial target pose",
                                                                targetPoseRed)),
                                        pathfindingCommandRed.until(
                                                () ->
                                                        Math.hypot(
                                                                                drive.getPose()
                                                                                                .getX()
                                                                                        - targetPoseRed
                                                                                                .getX(),
                                                                                drive.getPose()
                                                                                                .getY()
                                                                                        - targetPoseRed
                                                                                                .getY())
                                                                        <= switchingToSpecializedTranslationalTolerance
                                                                && Math.abs(
                                                                                drive.getRotation()
                                                                                                .getRadians()
                                                                                        - targetPoseRed
                                                                                                .getRotation()
                                                                                                .getRadians())
                                                                        <= switchingToSpecializedRotationalTolerance
                                                                && vision.seesFocusTag()),
                                        new GoToPositionSpecialized(
                                                drive,
                                                vision,
                                                config.position(),
                                                config.sidewaysOffset(),
                                                config.depthOffset(),
                                                reefAlgaeAlignmentConstraints),
                                        new ConditionalCommand(
                                                new ConditionalCommand(
                                                        Commands.runOnce(
                                                                () ->
                                                                        FieldSimulationManager
                                                                                .setCurrentObjective(
                                                                                        Objective
                                                                                                .UPPER_REEF_ALGAE)),
                                                        Commands.runOnce(
                                                                () ->
                                                                        FieldSimulationManager
                                                                                .setCurrentObjective(
                                                                                        Objective
                                                                                                .LOWER_REEF_ALGAE)),
                                                        () ->
                                                                height
                                                                        == ElevatorConstants
                                                                                .upperAlgaeHeight),
                                                Commands.runOnce(() -> {}),
                                                () -> Constants.currentMode == Mode.SIM)),
                                new IntakeReefAlgae(height, pivot, elevator, algaeClaw)),
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue),
                new ConditionalCommand(
                        Commands.runOnce(
                                () ->
                                        FieldSimulationManager.setCurrentObjective(
                                                Objective.NOTHING)),
                        Commands.runOnce(() -> {}),
                        () -> Constants.currentMode == Mode.SIM));
    }
}
