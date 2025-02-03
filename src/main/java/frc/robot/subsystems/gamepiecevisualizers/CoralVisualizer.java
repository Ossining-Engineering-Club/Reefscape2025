package frc.robot.subsystems.gamepiecevisualizers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class CoralVisualizer {
  public static enum CoralState {
    LOADED,
    GONE
  }

  public static CoralState coralState = CoralState.GONE;

  public static void setCoralState(CoralState coralState) {
    CoralVisualizer.coralState = coralState;
  }

  public static void update(Pose2d robotPose, double elevatorHeight, double pivotAngleRadians) {
    switch (CoralVisualizer.coralState) {
      case GONE:
        Logger.recordOutput(
            "CoralInRobot", new Pose3d(new Translation3d(0, 0, -1), new Rotation3d()));
        break;
      case LOADED:
        Logger.recordOutput(
            "CoralInRobot",
            new Pose3d(
                new Translation3d(
                    robotPose.getX()
                        + Math.cos(robotPose.getRotation().getRadians())
                            * (Units.inchesToMeters(11.471352)
                                + Math.cos(pivotAngleRadians) * Units.inchesToMeters(9.030342)),
                    robotPose.getY()
                        + Math.sin(robotPose.getRotation().getRadians())
                            * (Units.inchesToMeters(11.471352)
                                + Math.cos(pivotAngleRadians) * Units.inchesToMeters(9.030342)),
                    Units.inchesToMeters(24.826339)
                        + elevatorHeight
                        + Math.sin(pivotAngleRadians) * Units.inchesToMeters(9.030342)),
                new Rotation3d(
                    0.0,
                    -(pivotAngleRadians + Units.degreesToRadians(90)),
                    robotPose.getRotation().getRadians())));
      default:
        break;
    }
  }
}
