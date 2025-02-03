package frc.robot.subsystems.coralpivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class CoralPivotConstants {
  // motor constants
  public static final int canId = 0;
  public static final double motorReduction = 50.0 / 1.0;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final boolean isInverted = false;

  // control system
  public static final double kP = 1.0;
  public static final double kD = 0.0;
  public static final double kS = 0.1;
  public static final double kG = 0.1;
  public static final double simP = 10.0;
  public static final double simD = 0.0;
  public static final double simS = 0.0;
  public static final double simG = 0.257;
  public static final double pidTolerance = Units.degreesToRadians(0.5);

  // angle setpoints
  public static final double startAngle = Units.degreesToRadians(90);
  public static final double maxAngle = Units.degreesToRadians(90);
  public static final double minAngle = Units.degreesToRadians(-90);
  public static final double stowAngle = Units.degreesToRadians(90);
  public static final double intakeAngle = Units.degreesToRadians(-55.0);
  public static final double placeAngle = Units.degreesToRadians(65.0 /*55.0*/);
  public static final double reefAlgaeAngle = Units.degreesToRadians(-15.0);

  // sim settings
  public static final DCMotor gearbox = DCMotor.getFalcon500(1);
  public static final double lengthMeters = 0.3754247;
  public static final double massKg = 2.72155;
  public static final double pivotMOI = SingleJointedArmSim.estimateMOI(lengthMeters, massKg);
}
