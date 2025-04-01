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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.Level;
import frc.robot.AutoTeleopConstants.PositioningConfig;
import frc.robot.Constants;
import frc.robot.commands.gamepiecemanipulation.GoToPlacingCoralPosition;
import frc.robot.commands.pivot.PivotGoToAngle;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AutoPlaceCoralAuto extends SequentialCommandGroup {
    public AutoPlaceCoralAuto(
            PositioningConfig config,
            Level level,
            Pivot pivot,
            Elevator elevator,
            CoralHolder coralHolder,
            Drive drive,
            Vision vision,
            LED led)
            throws FileVersionException, IOException, ParseException {
        double sidewaysOffset;
        double depthOffset;
        if (level == Level.L1) {
            sidewaysOffset =
                    Math.signum(config.sidewaysOffset())
                                    * AutoTeleopConstants.sidewaysReefCoralL1Offset
                            - Constants.coralIntakeXOffset;
            depthOffset = config.depthOffset();
        } else if (level == Level.L2 || level == Level.L3) {
            sidewaysOffset = config.sidewaysOffset();
            depthOffset = AutoTeleopConstants.depthReefCoralL23Offset;
        } else {
            sidewaysOffset = config.sidewaysOffset();
            depthOffset = config.depthOffset();
        }
        Pose2d targetPoseBlue =
                GoToPositionSpecialized.getTargetPose(
                                getTagIdOfPosition(config.position(), Alliance.Blue),
                                sidewaysOffset,
                                depthOffset,
                                false)
                        .get();
        Pose2d targetPoseRed =
                GoToPositionSpecialized.getTargetPose(
                                getTagIdOfPosition(config.position(), Alliance.Red),
                                sidewaysOffset,
                                depthOffset,
                                false)
                        .get();
        Command pathfindingCommandBlue =
                AutoBuilder.pathfindToPose(
                        targetPoseBlue, reefCoralAutoPathfindingAlignmentConstraints, 0.0);
        Command pathfindingCommandRed =
                AutoBuilder.pathfindToPose(
                        targetPoseRed, reefCoralAutoPathfindingAlignmentConstraints, 0.0);

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
                    Commands.runOnce(() -> led.setIsPathfinding(true)),
                    Commands.runOnce(
                            () -> vision.setFocusTag(getTagIdOfPosition(config.position()))),
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    Commands.runOnce(
                                            () ->
                                                    Logger.recordOutput(
                                                            "initial target pose", targetPoseBlue)),
                                    new ParallelCommandGroup(
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
                                                                                    <= switchingToSpecializedRotationalTolerance
                                                                            && vision
                                                                                    .seesFocusTag()),
                                                    new GoToPositionSpecialized(
                                                            drive,
                                                            vision,
                                                            config.position(),
                                                            sidewaysOffset,
                                                            depthOffset,
                                                            reefCoralAutoPIDAlignmentConstraints)),
                                            new SequentialCommandGroup(
                                                    new WaitCommand(0.5),
                                                    new GoToPlacingCoralPosition(
                                                            height, level, pivot, elevator))),
                                    coralHolder.release(level),
                                    new PivotGoToAngle(pivot, PivotConstants.knockL4CoralAngle)),
                            new SequentialCommandGroup(
                                    Commands.runOnce(
                                            () ->
                                                    Logger.recordOutput(
                                                            "initial target pose", targetPoseRed)),
                                    new ParallelCommandGroup(
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
                                                                                    <= switchingToSpecializedRotationalTolerance
                                                                            && vision
                                                                                    .seesFocusTag()),
                                                    new GoToPositionSpecialized(
                                                            drive,
                                                            vision,
                                                            config.position(),
                                                            sidewaysOffset,
                                                            depthOffset,
                                                            reefCoralAutoPIDAlignmentConstraints)),
                                            new SequentialCommandGroup(
                                                    new WaitCommand(0.5),
                                                    new GoToPlacingCoralPosition(
                                                            height, level, pivot, elevator))),
                                    coralHolder.release(level),
                                    new PivotGoToAngle(pivot, PivotConstants.knockL4CoralAngle)),
                            () ->
                                    DriverStation.getAlliance().orElse(Alliance.Blue)
                                            == Alliance.Blue));
        } else {
            addCommands(
                    Commands.runOnce(() -> led.setIsPathfinding(true)),
                    Commands.runOnce(
                            () -> vision.setFocusTag(getTagIdOfPosition(config.position()))),
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    Commands.runOnce(
                                            () ->
                                                    Logger.recordOutput(
                                                            "initial target pose", targetPoseBlue)),
                                    new ParallelCommandGroup(
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
                                                                                    <= switchingToSpecializedRotationalTolerance
                                                                            && vision
                                                                                    .seesFocusTag()),
                                                    new GoToPositionSpecialized(
                                                            drive,
                                                            vision,
                                                            config.position(),
                                                            sidewaysOffset,
                                                            depthOffset,
                                                            reefCoralAutoPIDAlignmentConstraints)),
                                            new SequentialCommandGroup(
                                                    new WaitCommand(0.5),
                                                    new GoToPlacingCoralPosition(
                                                            height, level, pivot, elevator))),
                                    new WaitCommand(0.5),
                                    coralHolder.release(level)),
                            new SequentialCommandGroup(
                                    Commands.runOnce(
                                            () ->
                                                    Logger.recordOutput(
                                                            "initial target pose", targetPoseRed)),
                                    new ParallelCommandGroup(
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
                                                                                    <= switchingToSpecializedRotationalTolerance
                                                                            && vision
                                                                                    .seesFocusTag()),
                                                    new GoToPositionSpecialized(
                                                            drive,
                                                            vision,
                                                            config.position(),
                                                            sidewaysOffset,
                                                            depthOffset,
                                                            reefCoralAutoPIDAlignmentConstraints)),
                                            new SequentialCommandGroup(
                                                    new WaitCommand(0.5),
                                                    new GoToPlacingCoralPosition(
                                                            height, level, pivot, elevator))),
                                    new WaitCommand(0.5),
                                    coralHolder.release(level)),
                            () ->
                                    DriverStation.getAlliance().orElse(Alliance.Blue)
                                            == Alliance.Blue));
        }
    }
}
