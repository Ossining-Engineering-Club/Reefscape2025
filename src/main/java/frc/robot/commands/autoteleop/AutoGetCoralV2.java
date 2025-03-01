package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.coralStationAlignmentConstraints;
import static frc.robot.AutoTeleopConstants.getTagIdOfPosition;
import static frc.robot.AutoTeleopConstants.switchingToSpecializedRotationalTolerance;
import static frc.robot.AutoTeleopConstants.switchingToSpecializedTranslationalTolerance;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants.PositioningConfig;
import frc.robot.commands.gamepiecemanipulation.IntakeCoral;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;

public class AutoGetCoralV2 extends SequentialCommandGroup {
    public AutoGetCoralV2(
            PositioningConfig config,
            Pivot pivot,
            Elevator elevator,
            CoralHolder coralHolder,
            Drive drive,
            Vision vision) {
        Pose2d targetPose =
                GoToPosition.getTargetPose(
                                getTagIdOfPosition(config.position()),
                                config.sidewaysOffset(),
                                config.depthOffset())
                        .get();
        Command pathfindingCommand =
                AutoBuilder.pathfindToPose(targetPose, coralStationAlignmentConstraints, 0.0);

        addCommands(
                Commands.runOnce(() -> vision.setFocusTag(getTagIdOfPosition(config.position()))),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                pathfindingCommand.until(
                                        () ->
                                                Math.hypot(
                                                                        drive.getPose().getX()
                                                                                - targetPose.getX(),
                                                                        drive.getPose().getY()
                                                                                - targetPose.getY())
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
                                        coralStationAlignmentConstraints)),
                        new IntakeCoral(pivot, elevator, coralHolder)));
    }
}
