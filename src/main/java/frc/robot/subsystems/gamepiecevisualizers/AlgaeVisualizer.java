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
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

public class AlgaeVisualizer {
    public static enum AlgaeState {
        LOADED,
        GONE
    }

    public static AlgaeState algaeState = AlgaeState.GONE;

    private static Pose3d loadedAlgaePoseFieldRelative =
            new Pose3d(new Translation3d(0, 0, -1), new Rotation3d());
    private static Pose3d loadedAlgaePoseRobotRelative =
            new Pose3d(new Translation3d(0, 0, -1), new Rotation3d());
    private static Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(0));
    private static ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private static double elevatorHeight = 0.0;
    private static double pivotAngleRadians = 0.0;

    public static void setAlgaeState(AlgaeState algaeState) {
        AlgaeVisualizer.algaeState = algaeState;
    }

    public static void update(
            Pose2d robotPose,
            double elevatorHeight,
            double pivotAngleRadians,
            ChassisSpeeds chassisSpeeds) {
        AlgaeVisualizer.robotPose = robotPose;
        AlgaeVisualizer.chassisSpeeds = chassisSpeeds;
        AlgaeVisualizer.elevatorHeight = elevatorHeight;
        AlgaeVisualizer.pivotAngleRadians = pivotAngleRadians;
        switch (AlgaeVisualizer.algaeState) {
            case GONE:
                Logger.recordOutput(
                        "AlgaeInRobot", new Pose3d(new Translation3d(0, 0, -1), new Rotation3d()));
                break;
            case LOADED:
                Logger.recordOutput(
                        "AlgaeInRobot",
                        getLoadedAlgaePoseFieldRelative(
                                robotPose, elevatorHeight, pivotAngleRadians));
            default:
                break;
        }
        Logger.recordOutput("loadedAlgaePoseRobotRelative", loadedAlgaePoseRobotRelative);
        Logger.recordOutput("loadedAlgaePoseFieldRelative", loadedAlgaePoseFieldRelative);
    }

    public static void shootAlgae() {
        if (AlgaeVisualizer.algaeState == AlgaeState.LOADED && Constants.currentMode == Mode.SIM) {
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(
                            new ReefscapeAlgaeOnFly(
                                    robotPose.getTranslation(),
                                    new Translation2d(
                                            -loadedAlgaePoseRobotRelative.getY(),
                                            loadedAlgaePoseRobotRelative.getX()),
                                    chassisSpeeds,
                                    new Rotation2d(
                                            robotPose.getRotation().getRadians() - Math.PI / 2.0),
                                    Meters.of(loadedAlgaePoseRobotRelative.getZ()),
                                    MetersPerSecond.of(0.5),
                                    Radians.of(
                                            pivotAngleRadians
                                                    + Math.PI
                                                    - Units.degreesToRadians(50))));
            AlgaeVisualizer.algaeState = AlgaeState.GONE;
        }
    }

    public static Pose3d getLoadedAlgaePoseRobotRelative(
            Pose2d robotPose, double elevatorHeight, double pivotAngleRadians) {
        loadedAlgaePoseRobotRelative =
                new Pose3d(
                        new Translation3d(
                                0.085,
                                Math.cos(pivotAngleRadians + Math.PI) * Units.inchesToMeters(-20.5),
                                Units.inchesToMeters(31.2)
                                        + elevatorHeight
                                        + Math.sin(pivotAngleRadians + Math.PI)
                                                * Units.inchesToMeters(20.5)),
                        new Rotation3d(0.0, 0.0, 0.0));
        return loadedAlgaePoseRobotRelative;
    }

    public static Pose3d getLoadedAlgaePoseFieldRelative(
            Pose2d robotPose, double elevatorHeight, double pivotAngleRadians) {
        getLoadedAlgaePoseRobotRelative(robotPose, elevatorHeight, pivotAngleRadians);
        loadedAlgaePoseFieldRelative =
                new Pose3d(
                        new Translation3d(
                                robotPose.getX()
                                        + Math.cos(robotPose.getRotation().getRadians())
                                                * loadedAlgaePoseRobotRelative.getX()
                                        + Math.sin(-robotPose.getRotation().getRadians())
                                                * loadedAlgaePoseRobotRelative.getY(),
                                robotPose.getY()
                                        + Math.sin(robotPose.getRotation().getRadians())
                                                * loadedAlgaePoseRobotRelative.getX()
                                        + Math.cos(-robotPose.getRotation().getRadians())
                                                * loadedAlgaePoseRobotRelative.getY(),
                                loadedAlgaePoseRobotRelative.getZ()),
                        new Rotation3d(0.0, 0.0, 0.0));
        return loadedAlgaePoseFieldRelative;
    }

    public static boolean withinTolerance(double x, double target, double tolerance) {
        return Math.abs(x - target) <= tolerance;
    }
}
