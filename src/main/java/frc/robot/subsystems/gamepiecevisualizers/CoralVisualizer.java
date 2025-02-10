package frc.robot.subsystems.gamepiecevisualizers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class CoralVisualizer {
  public static enum CoralState {
    LOADED,
    GONE
  }

  public static CoralState coralState = CoralState.GONE;

  private static Pose3d loadedCoralPoseFieldRelative =
      new Pose3d(new Translation3d(0, 0, -1), new Rotation3d());
  private static Pose3d loadedCoralPoseRobotRelative =
      new Pose3d(new Translation3d(0, 0, -1), new Rotation3d());
  private static Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(0));
  private static ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  public static void setCoralState(CoralState coralState) {
    CoralVisualizer.coralState = coralState;
  }

  public static void update(
      Pose2d robotPose,
      double elevatorHeight,
      double pivotAngleRadians,
      ChassisSpeeds chassisSpeeds) {
    CoralVisualizer.robotPose = robotPose;
    CoralVisualizer.chassisSpeeds = chassisSpeeds;
    getLoadedCoralPoseFieldRelative(robotPose, elevatorHeight, pivotAngleRadians);
    switch (CoralVisualizer.coralState) {
      case GONE:
        Logger.recordOutput(
            "CoralInRobot", new Pose3d(new Translation3d(0, 0, -1), new Rotation3d()));
        break;
      case LOADED:
        Logger.recordOutput("CoralInRobot", loadedCoralPoseFieldRelative);
      default:
        break;
    }
    Logger.recordOutput("loadedCoralPoseRobotRelative", loadedCoralPoseRobotRelative);
    Logger.recordOutput("loadedCoralPoseFieldRelative", loadedCoralPoseFieldRelative);
  }

  public static void shootCoral() {
    if (CoralVisualizer.coralState == CoralState.LOADED) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new ReefscapeCoralOnFly(
                  robotPose.getTranslation(),
                  new Translation2d(
                      loadedCoralPoseRobotRelative.getX(), loadedCoralPoseRobotRelative.getY()),
                  chassisSpeeds,
                  new Rotation2d(loadedCoralPoseFieldRelative.getRotation().getZ() + Math.PI),
                  Meters.of(loadedCoralPoseRobotRelative.getZ()),
                  MetersPerSecond.of(1.5),
                  Radians.of(loadedCoralPoseRobotRelative.getRotation().getY())));
      CoralVisualizer.coralState = CoralState.GONE;
    }
  }

  public static Pose3d getLoadedCoralPoseRobotRelative(
      Pose2d robotPose, double elevatorHeight, double pivotAngleRadians) {
    loadedCoralPoseRobotRelative =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(-2.0),
                Math.cos(pivotAngleRadians) * Units.inchesToMeters(-13),
                Units.inchesToMeters(31.2)
                    + elevatorHeight
                    + Math.sin(pivotAngleRadians) * Units.inchesToMeters(13)),
            new Rotation3d(
                0.0,
                ((pivotAngleRadians + Math.PI / 2.0) % (Math.PI) - Math.PI / 2.0),
                Units.degreesToRadians(90)));
    return loadedCoralPoseRobotRelative;
  }

  public static Pose3d getLoadedCoralPoseFieldRelative(
      Pose2d robotPose, double elevatorHeight, double pivotAngleRadians) {
    getLoadedCoralPoseRobotRelative(robotPose, elevatorHeight, pivotAngleRadians);
    Logger.recordOutput("angle", loadedCoralPoseRobotRelative.getRotation().getY());
    loadedCoralPoseFieldRelative =
        new Pose3d(
            new Translation3d(
                robotPose.getX()
                    + Math.cos(robotPose.getRotation().getRadians())
                        * loadedCoralPoseRobotRelative.getX()
                    + Math.sin(-robotPose.getRotation().getRadians())
                        * loadedCoralPoseRobotRelative.getY(),
                robotPose.getY()
                    + Math.sin(robotPose.getRotation().getRadians())
                        * loadedCoralPoseRobotRelative.getX()
                    + Math.cos(-robotPose.getRotation().getRadians())
                        * loadedCoralPoseRobotRelative.getY(),
                loadedCoralPoseRobotRelative.getZ()),
            new Rotation3d(
                0.0,
                loadedCoralPoseRobotRelative.getRotation().getY(),
                robotPose.getRotation().getRadians()
                    + loadedCoralPoseRobotRelative.getRotation().getZ()));
    return loadedCoralPoseFieldRelative;
  }
}
