package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  // motor constants
  public static final int canId = 0;
  public static final double motorReduction = 25.0 / 1.0;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final boolean isInverted = false;

  // control system
  public static final double kP = 1.0;
  public static final double kD = 0.0;
  public static final double kS = 0.1;
  public static final double kG = 0.1;
  public static final double simP = 100.0;
  public static final double simD = 0.0;
  public static final double simS = 0.0;
  public static final double simG = 0.15;
  public static final double pidTolerance = 0.002; // meters

  // angle setpoints
  public static final double startHeight = 0.0;
  public static final double maxHeight = 1.25;
  public static final double minHeight = 0.0;
  public static final double l1Height = 0.0;
  public static final double l2Height = Units.inchesToMeters(0.255);
  public static final double l3Height = Units.inchesToMeters(16.325);
  public static final double l4Height = Units.inchesToMeters(45.735);
  public static final double coralIntakeHeight = Units.inchesToMeters(17.744);
  public static final double groundAlgaeHeight = 0.0;
  public static final double lowerAlgaeHeight = Units.inchesToMeters(29.945);
  public static final double upperAlgaeHeight = Units.inchesToMeters(41.602);
  public static final double processorHeight = 0.0;

  // sim settings
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);
  public static final double massKg = 18.1437;
  public static final double drumRadiusMeters = Units.inchesToMeters(1.0);
}
