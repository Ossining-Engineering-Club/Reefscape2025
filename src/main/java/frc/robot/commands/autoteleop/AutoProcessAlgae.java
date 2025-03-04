package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.*;

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
import frc.robot.commands.gamepiecemanipulation.GoToProcessingPosition;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AutoProcessAlgae extends SequentialCommandGroup {
    public AutoProcessAlgae(
            PositioningConfig config,
            Pivot pivot,
            Elevator elevator,
            AlgaeClaw algaeClaw,
            Drive drive,
            LED led)
            throws FileVersionException, IOException, ParseException {
        Pose2d targetPoseBlue =
                GoToPositionSpecialized.getTargetPose(
                                getTagIdOfPosition(config.position(), Alliance.Blue),
                                config.sidewaysOffset(),
                                config.depthOffset(),
                                true)
                        .get();
        Pose2d targetPoseRed =
                GoToPositionSpecialized.getTargetPose(
                                getTagIdOfPosition(config.position(), Alliance.Red),
                                config.sidewaysOffset(),
                                config.depthOffset(),
                                true)
                        .get();
        Command pathfindingCommandBlue =
                AutoBuilder.pathfindToPose(
                        targetPoseBlue, processorPathfindingAlignmentConstraints, 0.0);
        Command pathfindingCommandRed =
                AutoBuilder.pathfindToPose(
                        targetPoseRed, processorPathfindingAlignmentConstraints, 0.0);

        addCommands(
                Commands.runOnce(() -> led.setIsPathfinding(true)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                Commands.runOnce(
                                        () ->
                                                Logger.recordOutput(
                                                        "initial target pose", targetPoseBlue)),
                                new ParallelCommandGroup(
                                        new GoToProcessingPosition(pivot, elevator),
                                        new SequentialCommandGroup(
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
                                                                                <= switchingToSpecializedRotationalTolerance),
                                                new GoToPositionGlobal(
                                                        drive,
                                                        config.position(),
                                                        config.sidewaysOffset(),
                                                        config.depthOffset(),
                                                        processorPIDAlignmentConstraints))),
                                algaeClaw.release()),
                        new SequentialCommandGroup(
                                Commands.runOnce(
                                        () ->
                                                Logger.recordOutput(
                                                        "initial target pose", targetPoseRed)),
                                new ParallelCommandGroup(
                                        new GoToProcessingPosition(pivot, elevator),
                                        new SequentialCommandGroup(
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
                                                                                <= switchingToSpecializedRotationalTolerance),
                                                new GoToPositionGlobal(
                                                        drive,
                                                        config.position(),
                                                        config.sidewaysOffset(),
                                                        config.depthOffset(),
                                                        processorPIDAlignmentConstraints))),
                                algaeClaw.release()),
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue));
    }
}
