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
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.PivotConstants;
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
    private static double elevatorHeight = 0.0;
    private static double pivotAngleRadians = 0.0;

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
        CoralVisualizer.elevatorHeight = elevatorHeight;
        CoralVisualizer.pivotAngleRadians = pivotAngleRadians;
        switch (CoralVisualizer.coralState) {
            case GONE:
                Logger.recordOutput(
                        "CoralInRobot", new Pose3d(new Translation3d(0, 0, -1), new Rotation3d()));
                break;
            case LOADED:
                Logger.recordOutput(
                        "CoralInRobot",
                        getLoadedCoralPoseFieldRelative(
                                robotPose, elevatorHeight, pivotAngleRadians));
            default:
                break;
        }
        Logger.recordOutput("loadedCoralPoseRobotRelative", loadedCoralPoseRobotRelative);
        Logger.recordOutput("loadedCoralPoseFieldRelative", loadedCoralPoseFieldRelative);
    }

    public static void shootCoral(boolean isL1) {
        if (CoralVisualizer.coralState == CoralState.LOADED && Constants.currentMode == Mode.SIM) {
            if (isL1) {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                                new ReefscapeCoralOnFly(
                                        robotPose.getTranslation(),
                                        new Translation2d(
                                                -loadedCoralPoseRobotRelative.getY(),
                                                loadedCoralPoseRobotRelative.getX()),
                                        chassisSpeeds,
                                        new Rotation2d(
                                                loadedCoralPoseFieldRelative.getRotation().getZ()
                                                        + Math.PI),
                                        Meters.of(loadedCoralPoseRobotRelative.getZ()),
                                        MetersPerSecond.of(1.0),
                                        Radians.of(
                                                loadedCoralPoseRobotRelative
                                                        .getRotation()
                                                        .getY())));
            } else if (withinTolerance(
                            elevatorHeight,
                            ElevatorConstants.l4Height,
                            5.0 * ElevatorConstants.pidTolerance)
                    && withinTolerance(
                            pivotAngleRadians,
                            PivotConstants.placeL4CoralAngle,
                            5.0 * PivotConstants.pidTolerance)) {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                                new ReefscapeCoralOnFly(
                                        robotPose.getTranslation(),
                                        new Translation2d(
                                                -loadedCoralPoseRobotRelative.getY() + 0.2,
                                                loadedCoralPoseRobotRelative.getX()),
                                        chassisSpeeds,
                                        new Rotation2d(
                                                loadedCoralPoseFieldRelative.getRotation().getZ()
                                                        + Math.PI),
                                        Meters.of(loadedCoralPoseRobotRelative.getZ()),
                                        MetersPerSecond.of(2),
                                        Radians.of(-Math.PI / 2.0)));
            } else {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                                new ReefscapeCoralOnFly(
                                        robotPose.getTranslation(),
                                        new Translation2d(
                                                -loadedCoralPoseRobotRelative.getY(),
                                                loadedCoralPoseRobotRelative.getX()),
                                        chassisSpeeds,
                                        new Rotation2d(
                                                loadedCoralPoseFieldRelative.getRotation().getZ()
                                                        + Math.PI),
                                        Meters.of(loadedCoralPoseRobotRelative.getZ()),
                                        MetersPerSecond.of(4),
                                        Radians.of(
                                                loadedCoralPoseRobotRelative
                                                        .getRotation()
                                                        .getY())));
            }
            CoralVisualizer.coralState = CoralState.GONE;
        }
    }

    public static Pose3d getLoadedCoralPoseRobotRelative(
            Pose2d robotPose, double elevatorHeight, double pivotAngleRadians) {
        loadedCoralPoseRobotRelative =
                new Pose3d(
                        new Translation3d(
                                -0.023,
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

    public static boolean withinTolerance(double x, double target, double tolerance) {
        return Math.abs(x - target) <= tolerance;
    }
}
