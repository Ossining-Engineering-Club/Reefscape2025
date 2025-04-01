package frc.robot.commands.autoteleop;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants;
import frc.robot.commands.gamepiecemanipulation.GoToNetAlgaePosition;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

public class AutoNetAlgaeAuto extends SequentialCommandGroup {
    public AutoNetAlgaeAuto(Drive drive, Elevator elevator, Pivot pivot, AlgaeClaw algaeClaw) {
        Pose2d targetPoseBlue = new Pose2d(7.663, 5.039, Rotation2d.fromDegrees(90));
        Pose2d targetPoseRed = new Pose2d(9.925, 3.021, Rotation2d.fromDegrees(-90));
        Command pathfindingCommandBlue = AutoBuilder.pathfindToPose(targetPoseBlue, AutoTeleopConstants.netAlignmentConstraints);
        Command pathfindingCommandRed = AutoBuilder.pathfindToPose(targetPoseRed, AutoTeleopConstants.netAlignmentConstraints);
        // addCommands(
        //     new ConditionalCommand(
        //         new SequentialCommandGroup(
        //             new ParallelCommandGroup(pathfindingCommandBlue, new GoToNetAlgaePosition(pivot, elevator)),
        //             algaeClaw
        //         ),
        //         new SequentialCommandGroup(),
        //             () ->
        //                     DriverStation.getAlliance().orElse(Alliance.Blue)
        //                             == Alliance.Blue)
        // );
    }
}
