package frc.robot.commands.autoteleop;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants;
import frc.robot.commands.gamepiecemanipulation.GoToNetAlgaePosition;
import frc.robot.commands.gamepiecemanipulation.ReleaseAlgae;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

public class AutoNetAlgae extends SequentialCommandGroup {
  public AutoNetAlgae(Pivot pivot, Elevator elevator, AlgaeClaw algaeClaw, Pose2d robotPose) {
    Pose2d targetPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      double y = Math.max(4.45, Math.min(7.783, robotPose.getY()));
      targetPose = new Pose2d(7.82, y, Rotation2d.fromDegrees(90));
    } else {
      double y = Math.max(0.554, Math.min(3.61, robotPose.getY()));
      targetPose = new Pose2d(9.73, y, Rotation2d.fromDegrees(-90));
    }
    Command pathFindingCommand =
        AutoBuilder.pathfindToPose(targetPose, AutoTeleopConstants.netAlignmentConstraints, 0.0);

    if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            && robotPose.getX() > 7.82)
        || (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            && robotPose.getX() < 9.73)) {
      addCommands(
          pathFindingCommand,
          new GoToNetAlgaePosition(pivot, elevator, algaeClaw),
          new ReleaseAlgae(algaeClaw));
    } else {
      addCommands(
          new ParallelCommandGroup(
              pathFindingCommand, new GoToNetAlgaePosition(pivot, elevator, algaeClaw)),
          new ReleaseAlgae(algaeClaw));
    }
  }
}
