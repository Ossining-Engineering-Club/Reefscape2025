package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClaw.AlgaeClawState;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolder.CoralHolderState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gamepiecevisualizers.AlgaeVisualizer;
import frc.robot.subsystems.gamepiecevisualizers.AlgaeVisualizer.AlgaeState;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer.CoralState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class FieldSimulationManager {
    public static enum Objective {
        NOTHING,
        UPPER_REEF_ALGAE,
        LOWER_REEF_ALGAE,
    }

    private static final Rotation2d coralStationLeftBlueRotation = Rotation2d.fromDegrees(-144);
    private static final Rotation2d coralStationRightBlueRotation = Rotation2d.fromDegrees(-36);
    private static final Rotation2d coralStationLeftRedRotation = Rotation2d.fromDegrees(36);
    private static final Rotation2d coralStationRightRedRotation = Rotation2d.fromDegrees(144);
    private static final Rotation2d coralStationRotationTolerance = Rotation2d.fromDegrees(20);

    private static final Translation2d[] coralStationLeftBlueDropArea =
            new Translation2d[] {
                new Translation2d(0.585, 6.784),
                new Translation2d(0.722, 6.589),
                new Translation2d(1.716, 7.301),
                new Translation2d(1.55, 7.515)
            };

    private static final Translation2d[] coralStationRightBlueDropArea =
            new Translation2d[] {
                new Translation2d(0.985, 1.617),
                new Translation2d(0.546, 1.198),
                new Translation2d(1.492, 0.534),
                new Translation2d(1.911, 0.983)
            };

    private static final Translation2d[] coralStationLeftRedDropArea =
            new Translation2d[] {
                new Translation2d(15.697, 0.983),
                new Translation2d(16.000, 0.534),
                new Translation2d(16.975, 1.227),
                new Translation2d(16.624, 1.617)
            };

    private static final Translation2d[] coralStationRightRedDropArea =
            new Translation2d[] {
                new Translation2d(16.731, 6.394),
                new Translation2d(16.975, 6.804),
                new Translation2d(16.009, 7.554),
                new Translation2d(15.658, 7.194)
            };

    private static double secondsPerCoralDrop = 2.0;
    private static double secondsSinceLastCoralDrop = 10000;

    private static Objective currentObjective = Objective.NOTHING;

    public static void periodic(
            Pose2d robotPose,
            Elevator elevator,
            Pivot pivot,
            CoralHolder coralHolder,
            AlgaeClaw algaeClaw) {
        Logger.recordOutput(
                "within area",
                withinArea(robotPose.getTranslation(), coralStationLeftBlueDropArea)
                        || withinArea(robotPose.getTranslation(), coralStationRightBlueDropArea)
                        || withinArea(robotPose.getTranslation(), coralStationLeftRedDropArea)
                        || withinArea(robotPose.getTranslation(), coralStationRightRedDropArea));
        Logger.recordOutput(
                "within rotation",
                withinRotationTolerance(
                                robotPose.getRotation(),
                                coralStationLeftBlueRotation,
                                coralStationRotationTolerance)
                        || withinRotationTolerance(
                                robotPose.getRotation(),
                                coralStationRightBlueRotation,
                                coralStationRotationTolerance)
                        || withinRotationTolerance(
                                robotPose.getRotation(),
                                coralStationLeftRedRotation,
                                coralStationRotationTolerance)
                        || withinRotationTolerance(
                                robotPose.getRotation(),
                                coralStationRightRedRotation,
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
                && ((withinArea(robotPose.getTranslation(), coralStationLeftBlueDropArea)
                                && withinRotationTolerance(
                                        robotPose.getRotation(),
                                        coralStationLeftBlueRotation,
                                        coralStationRotationTolerance))
                        || (withinArea(robotPose.getTranslation(), coralStationRightBlueDropArea)
                                && withinRotationTolerance(
                                        robotPose.getRotation(),
                                        coralStationRightBlueRotation,
                                        coralStationRotationTolerance))
                        || (withinArea(robotPose.getTranslation(), coralStationLeftRedDropArea)
                                && withinRotationTolerance(
                                        robotPose.getRotation(),
                                        coralStationLeftRedRotation,
                                        coralStationRotationTolerance))
                        || (withinArea(robotPose.getTranslation(), coralStationRightRedDropArea)
                                && withinRotationTolerance(
                                        robotPose.getRotation(),
                                        coralStationRightRedRotation,
                                        coralStationRotationTolerance)))
                && withinTolerance(
                        elevator.getHeight(),
                        ElevatorConstants.intakeCoralHeight,
                        5 * ElevatorConstants.pidTolerance)
                && (withinTolerance(
                                pivot.getAngle(),
                                PivotConstants.intakeCoralAngle,
                                5 * PivotConstants.pidTolerance)
                        || withinTolerance(
                                pivot.getAngle(),
                                PivotConstants.intakeCoralAutoAngle,
                                5 * PivotConstants.pidTolerance))
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
            CoralVisualizer.setCoralState(CoralState.LOADED);
            secondsSinceLastCoralDrop = 0;
        } else {
            secondsSinceLastCoralDrop += 0.02;
        }

        if (withinTolerance(
                        pivot.getAngle(),
                        PivotConstants.intakeReefAlgaeAngle,
                        5 * PivotConstants.pidTolerance)
                && ((getCurrentObjective() == Objective.UPPER_REEF_ALGAE
                                && withinTolerance(
                                        elevator.getHeight(),
                                        ElevatorConstants.upperAlgaeHeight,
                                        5 * ElevatorConstants.pidTolerance))
                        || (getCurrentObjective() == Objective.LOWER_REEF_ALGAE
                                && withinTolerance(
                                        elevator.getHeight(),
                                        ElevatorConstants.lowerAlgaeHeight,
                                        5 * ElevatorConstants.pidTolerance)))
                && algaeClaw.getState() == AlgaeClawState.FORWARD) {
            AlgaeVisualizer.setAlgaeState(AlgaeState.LOADED);
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

    public static Objective getCurrentObjective() {
        return currentObjective;
    }

    public static void setCurrentObjective(Objective objective) {
        currentObjective = objective;
    }
}
