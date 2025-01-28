package frc.robot.subsystems.groundintakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GroundIntakePivotConstants {
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
  public static final double simG = 0.2;
  public static final double pidTolerance = Units.degreesToRadians(0.5);

  // angle setpoints
  public static final double startAngle = Units.degreesToRadians(80);
  public static final double maxAngle = Units.degreesToRadians(80);
  public static final double minAngle = 0.0;
  public static final double stowAngle = Units.degreesToRadians(70);
  public static final double extendAngle = Units.degreesToRadians(35);
  public static final double avoidanceAngle = Units.degreesToRadians(20);

  // sim settings
  public static final DCMotor gearbox = DCMotor.getFalcon500(1);
  public static final double lengthMeters = 0.536575;
  public static final double massKg = 1.4723608;
  public static final double pivotMOI = SingleJointedArmSim.estimateMOI(lengthMeters, massKg);
}
