package frc.robot.commands.autoteleop;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants;
import frc.robot.FieldConstants;
import frc.robot.commands.gamepiecemanipulation.GoToNetAlgaePosition;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;

public class AutoNetAlgaeAuto extends SequentialCommandGroup {
    private final Pose2d targetPoseBlue =
            new Pose2d(
                    FieldConstants.centerX - 1.0 /*7.636*/,
                    FieldConstants.centerY + 1.0 /*5.098*/,
                    Rotation2d.fromDegrees(90.0));
    private final Pose2d targetPoseRed =
            new Pose2d(
                    FieldConstants.centerX + 1.0 /*9.950*/,
                    FieldConstants.centerY - 1.0 /*3.036*/,
                    Rotation2d.fromDegrees(-90.0));

    public AutoNetAlgaeAuto(
            Drive drive, Elevator elevator, Pivot pivot, AlgaeClaw algaeClaw, LED led) {
        Command pathfindingCommandBlue =
                AutoBuilder.pathfindToPose(
                        targetPoseBlue, AutoTeleopConstants.netAlignmentConstraints, 0.0);
        Command pathfindingCommandRed =
                AutoBuilder.pathfindToPose(
                        targetPoseRed, AutoTeleopConstants.netAlignmentConstraints, 0.0);

        addCommands(
                Commands.runOnce(() -> led.setIsPathfinding(true)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        pathfindingCommandBlue,
                                        new GoToNetAlgaePosition(pivot, elevator)),
                                algaeClaw.release()),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        pathfindingCommandRed,
                                        new GoToNetAlgaePosition(pivot, elevator)),
                                algaeClaw.release()),
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue));
    }
}
