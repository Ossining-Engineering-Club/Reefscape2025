package frc.robot.subsystems.coralpivot;

import edu.wpi.first.math.util.Units;

public class CoralPivotConstants {
  // motor constants
  public static final int canId = 0;
  public static final double motorReduction = 1.0 / 1.0;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final boolean isInverted = false;

  // control system
  public static final double kP = 1.0;
  public static final double kD = 0.0;
  public static final double kS = 0.1;
  public static final double kG = 0.1;
  public static final double pidTolerance = Units.degreesToRadians(0.5);

  // angle setpoints
  public static final double startAngle = 0.0;
  public static final double maxAngle = 0.0;
  public static final double minAngle = 0.0;
  public static final double stowAngle = 0.0;
  public static final double extendAngle = 0.0;
}
