package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.getTagIdOfPosition;
import static frc.robot.AutoTeleopConstants.reefCoralAlignmentConstraints;
import static frc.robot.AutoTeleopConstants.switchingToSpecializedRotationalTolerance;
import static frc.robot.AutoTeleopConstants.switchingToSpecializedTranslationalTolerance;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoTeleopConstants.Level;
import frc.robot.AutoTeleopConstants.PositioningConfig;
import frc.robot.commands.gamepiecemanipulation.GoToPlacingCoralPosition;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoPlaceCoralV2 extends SequentialCommandGroup {
    public AutoPlaceCoralV2(
            PositioningConfig config,
            Level level,
            Pivot pivot,
            Elevator elevator,
            CoralHolder coralHolder,
            Drive drive,
            Vision vision)
            throws FileVersionException, IOException, ParseException {

        Pose2d targetPose =
                GoToPosition.getTargetPose(
                                getTagIdOfPosition(config.position()),
                                config.sidewaysOffset(),
                                config.depthOffset())
                        .get();
        Command pathfindingCommand =
                AutoBuilder.pathfindToPose(targetPose, reefCoralAlignmentConstraints, 0.0);

        double height =
                switch (level) {
                    case L1 -> ElevatorConstants.l1Height;
                    case L2 -> ElevatorConstants.l2Height;
                    case L3 -> ElevatorConstants.l3Height;
                    case L4 -> ElevatorConstants.l4Height;
                    default -> 0.0;
                };

        if (level == Level.L4) {
            addCommands(
                    Commands.runOnce(
                            () -> vision.setFocusTag(getTagIdOfPosition(config.position()))),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    pathfindingCommand.until(
                                            () ->
                                                    Math.hypot(
                                                                            drive.getPose().getX()
                                                                                    - targetPose
                                                                                            .getX(),
                                                                            drive.getPose().getY()
                                                                                    - targetPose
                                                                                            .getY())
                                                                    <= switchingToSpecializedTranslationalTolerance
                                                            && Math.abs(
                                                                            drive.getRotation()
                                                                                            .getRadians()
                                                                                    - targetPose
                                                                                            .getRotation()
                                                                                            .getRadians())
                                                                    <= switchingToSpecializedRotationalTolerance
                                                            && vision.seesFocusTag()),
                                    new GoToPosition(
                                            drive,
                                            vision,
                                            config.position(),
                                            config.sidewaysOffset(),
                                            config.depthOffset(),
                                            reefCoralAlignmentConstraints)),
                            new SequentialCommandGroup(
                                    new WaitCommand(1.5),
                                    new GoToPlacingCoralPosition(height, level, pivot, elevator))),
                    coralHolder.release(),
                    new PivotGoToAngle(pivot, PivotConstants.knockL4CoralAngle));
        } else {
            addCommands(
                    Commands.runOnce(
                            () -> vision.setFocusTag(getTagIdOfPosition(config.position()))),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    pathfindingCommand.until(
                                            () ->
                                                    Math.hypot(
                                                                            drive.getPose().getX()
                                                                                    - targetPose
                                                                                            .getX(),
                                                                            drive.getPose().getY()
                                                                                    - targetPose
                                                                                            .getY())
                                                                    <= switchingToSpecializedTranslationalTolerance
                                                            && Math.abs(
                                                                            drive.getRotation()
                                                                                            .getRadians()
                                                                                    - targetPose
                                                                                            .getRotation()
                                                                                            .getRadians())
                                                                    <= switchingToSpecializedRotationalTolerance
                                                            && vision.seesFocusTag()),
                                    new GoToPosition(
                                            drive,
                                            vision,
                                            config.position(),
                                            config.sidewaysOffset(),
                                            config.depthOffset(),
                                            reefCoralAlignmentConstraints)),
                            new SequentialCommandGroup(
                                    new WaitCommand(1.5),
                                    new GoToPlacingCoralPosition(height, level, pivot, elevator))),
                    coralHolder.release());
        }
    }
}
