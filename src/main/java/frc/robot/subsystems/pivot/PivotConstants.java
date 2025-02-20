package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class PivotConstants {
  // motor constants
  public static final int canId = 17;
  public static final double motorReduction = 50.0 / 1.0;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final boolean isInverted = false;
  public static final int currentLimit = 30;

  // control system
  public static final double kP = 1.0;
  public static final double kD = 0.0;
  public static final double maxVelocity = Units.degreesToRadians(360.0);
  public static final double maxAcceleration = Units.degreesToRadians(360.0);
  public static final double simP = 10.0;
  public static final double simD = 1.25;
  public static final double simMaxVelocity = Units.degreesToRadians(720.0);
  public static final double simMaxAcceleration = Units.degreesToRadians(720.0);
  public static final double pidTolerance = Units.degreesToRadians(0.5);

  // angle setpoints
  public static final double startAngle = Units.degreesToRadians(53.763);
  public static final double maxAngle = Units.degreesToRadians(90);
  public static final double minAngle = Units.degreesToRadians(-135.0);
  public static final double stowAngle = Units.degreesToRadians(0);
  public static final double intakeCoralAngle = Units.degreesToRadians(35.0);
  public static final double placeCoralAngle = Units.degreesToRadians(-35.0);
  public static final double intakeReefAlgaeAngle = Units.degreesToRadians(30.0);
  public static final double processorAngle = Units.degreesToRadians(35.0);
  public static final double netAngle = Units.degreesToRadians(-135.0);

  // sim settings
  public static final DCMotor gearbox = DCMotor.getFalcon500(1);
  public static final double pivotMOI = 2.058176725;
}
