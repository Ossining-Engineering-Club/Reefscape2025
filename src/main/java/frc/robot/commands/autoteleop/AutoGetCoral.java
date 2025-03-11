package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants.PositioningConfig;
import frc.robot.commands.gamepiecemanipulation.IntakeCoralAuto;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

// ONLY USE FOR AUTO
public class AutoGetCoral extends SequentialCommandGroup {
    public AutoGetCoral(
            PositioningConfig config,
            Pivot pivot,
            Elevator elevator,
            CoralHolder coralHolder,
            Drive drive,
            Vision vision,
            LED led) {
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
                AutoBuilder.pathfindToPose(
                        targetPoseBlue, coralStationPathfindingAlignmentConstraints, 0.0);
        Command pathfindingCommandRed =
                AutoBuilder.pathfindToPose(
                        targetPoseRed, coralStationPathfindingAlignmentConstraints, 0.0);

        addCommands(
                Commands.runOnce(() -> led.setIsPathfinding(true)),
                Commands.runOnce(() -> vision.setFocusTag(getTagIdOfPosition(config.position()))),
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
                                                coralStationPIDAlignmentConstraints)),
                                new IntakeCoralAuto(pivot, elevator, coralHolder)),
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
                                                coralStationPIDAlignmentConstraints)),
                                new IntakeCoralAuto(pivot, elevator, coralHolder)),
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue));
    }
}
