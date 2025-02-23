package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolder.CoralHolderState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer.CoralState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class FieldSimulationManager {
    private static final Rotation2d coralStationLeftRotation = Rotation2d.fromDegrees(-144);
    private static final Rotation2d coralStationRightRotation = Rotation2d.fromDegrees(-36);
    private static final Rotation2d coralStationRotationTolerance = Rotation2d.fromDegrees(5);

    private static final Translation2d[] coralStationLeftDropArea =
            new Translation2d[] {
                new Translation2d(0.585, 6.784),
                new Translation2d(0.722, 6.589),
                new Translation2d(1.716, 7.301),
                new Translation2d(1.55, 7.515)
            };

    private static final Translation2d[] coralStationRightDropArea =
            new Translation2d[] {
                new Translation2d(0.702, 1.412),
                new Translation2d(0.546, 1.198),
                new Translation2d(1.492, 0.534),
                new Translation2d(1.736, 0.700)
            };

    private static double secondsPerCoralDrop = 2.0;
    private static double secondsSinceLastCoralDrop = 10000;

    public static void periodic(
            Pose2d robotPose, Elevator elevator, Pivot pivot, CoralHolder coralHolder) {
        Logger.recordOutput(
                "within area", withinArea(robotPose.getTranslation(), coralStationLeftDropArea));
        Logger.recordOutput(
                "within rotation",
                withinRotationTolerance(
                        robotPose.getRotation(),
                        coralStationLeftRotation,
                        coralStationRotationTolerance));
        Logger.recordOutput(
                "within elevator",
                withinTolerance(
                        elevator.getHeight(),
                        ElevatorConstants.intakeCoralHeight,
                        5 * ElevatorConstants.pidTolerance));
        Logger.recordOutput(
                "within pivot",
                withinTolerance(
                        pivot.getAngle(),
                        PivotConstants.intakeCoralAngle,
                        5 * PivotConstants.pidTolerance));
        Logger.recordOutput(
                "within holder state", coralHolder.getState() == CoralHolderState.FORWARD);
        Logger.recordOutput("within coral state", CoralVisualizer.coralState == CoralState.GONE);
        if (secondsSinceLastCoralDrop >= secondsPerCoralDrop
                && ((withinArea(robotPose.getTranslation(), coralStationLeftDropArea)
                                && withinRotationTolerance(
                                        robotPose.getRotation(),
                                        coralStationLeftRotation,
                                        coralStationRotationTolerance))
                        || (withinArea(robotPose.getTranslation(), coralStationRightDropArea)
                                && withinRotationTolerance(
                                        robotPose.getRotation(),
                                        coralStationRightRotation,
                                        coralStationRotationTolerance)))
                && withinTolerance(
                        elevator.getHeight(),
                        ElevatorConstants.intakeCoralHeight,
                        5 * ElevatorConstants.pidTolerance)
                && withinTolerance(
                        pivot.getAngle(),
                        PivotConstants.intakeCoralAngle,
                        5 * PivotConstants.pidTolerance)
                && coralHolder.getState() == CoralHolderState.FORWARD
                && CoralVisualizer.coralState == CoralState.GONE) {
            // SimulatedArena.getInstance()
            //     .addGamePieceProjectile(
            //         new ReefscapeCoralOnFly(
            //             robotPose.getTranslation(),
            //             new Translation2d(0.5, 0),
            //             new ChassisSpeeds(0, 0, 0),
            //             coralStationLeftRotation,
            //             Meters.of(1.05),
            //             MetersPerSecond.of(-2),
            //             Degrees.of(35)));
            CoralVisualizer.coralState = CoralState.LOADED;
            secondsSinceLastCoralDrop = 0;
        } else {
            secondsSinceLastCoralDrop += 0.02;
        }
    }

    public static boolean withinArea(Translation2d point, Translation2d[] area) {
        boolean ret = true;
        for (int i = 0; i < area.length; i++) {
            if ((area[(i + 1) % area.length].getX() - area[i].getX())
                                    * (point.getY() - area[i].getY())
                            - (point.getX() - area[i].getX())
                                    * (area[(i + 1) % area.length].getY() - area[i].getY())
                    < 0) {
                ret = false;
            }
        }
        return ret;
    }

    public static boolean withinRotationTolerance(
            Rotation2d rotation, Rotation2d target, Rotation2d tolerance) {
        return Math.abs(rotation.minus(target).getRadians()) <= tolerance.getRadians();
    }

    public static boolean withinTolerance(double x, double target, double tolerance) {
        return Math.abs(x - target) <= tolerance;
    }
}
