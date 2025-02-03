package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolder.CoralHolderState;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer.CoralState;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class FieldSimulationManager {
  private static final Rotation2d coralStationLeftRotation = Rotation2d.fromDegrees(126);
  private static final Rotation2d coralStationRightRotation = Rotation2d.fromDegrees(-126);
  private static final Rotation2d coralStationRotationTolerance = Rotation2d.fromDegrees(5);

  private static final Translation2d[] coralStationLeftDropArea =
      new Translation2d[] {
        new Translation2d(0.585, 6.784),
        new Translation2d(0.722, 6.589),
        new Translation2d(1.716, 7.301),
        new Translation2d(1.55, 7.515)
      };

  private static double secondsPerCoralDrop = 2.0;
  private static double secondsSinceLastCoralDrop = 10000;

  public static void periodic(
      Pose2d robotPose, Elevator elevator, CoralPivot coralPivot, CoralHolder coralHolder) {
    if (secondsSinceLastCoralDrop >= secondsPerCoralDrop
        && withinArea(robotPose.getTranslation(), coralStationLeftDropArea)
        && withinRotationTolerance(
            robotPose.getRotation(), coralStationLeftRotation, coralStationRotationTolerance)
        && withinTolerance(
            elevator.getHeight(),
            ElevatorConstants.coralIntakeHeight,
            5 * ElevatorConstants.pidTolerance)
        && withinTolerance(
            coralPivot.getAngle(),
            CoralPivotConstants.intakeAngle,
            5 * CoralPivotConstants.pidTolerance)
        && coralHolder.getState() == CoralHolderState.FORWARD
        && CoralVisualizer.coralState == CoralState.GONE) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new ReefscapeCoralOnFly(
                  robotPose.getTranslation(),
                  new Translation2d(0.5, 0),
                  new ChassisSpeeds(0, 0, 0),
                  coralStationLeftRotation,
                  Meters.of(1.05),
                  MetersPerSecond.of(-2),
                  Degrees.of(35)));
      secondsSinceLastCoralDrop = 0;
    } else {
      secondsSinceLastCoralDrop += 0.02;
    }
  }

  public static boolean withinArea(Translation2d point, Translation2d[] area) {
    boolean ret = true;
    for (int i = 0; i < area.length; i++) {
      if ((area[(i + 1) % area.length].getX() - area[i].getX()) * (point.getY() - area[i].getY())
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
