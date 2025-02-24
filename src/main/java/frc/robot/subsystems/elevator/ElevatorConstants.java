package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    // motor constants
    public static final int canId = 16;
    public static final double motorReduction = 25.0 / 1.0;
    public static final boolean isInverted = false;
    public static final int stallCurrentLimit = 30;
    public static final int freeCurrentLimit = 50;

    // control system
    public static final double kP = 150.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.25;
    public static final double maxVelocity = 0.75; // m/s
    public static final double maxAcceleration = 1.5; // m/s^2
    public static final double simP = 200.0;
    public static final double simD = 0.0;
    public static final double simS = 0.0;
    public static final double simG = 0.15 / 2.0;
    public static final double simMaxVelocity = 0.75; // m/s
    public static final double simMaxAcceleration = 1.5; // m/s^2
    public static final double pidTolerance = 0.002; // meters

    // height setpoints
    public static final double startHeight = 0.0;
    public static final double maxHeight = 1.169;
    public static final double minHeight = 0.0;
    public static final double l1Height = Units.inchesToMeters(3.0);
    public static final double l2Height = Units.inchesToMeters(14.0);
    public static final double l3Height = Units.inchesToMeters(30.0);
    public static final double l4Height = Units.inchesToMeters(53);
    public static final double intakeCoralHeight = Units.inchesToMeters(0);
    public static final double lowerAlgaeHeight = Units.inchesToMeters(16);
    public static final double upperAlgaeHeight = Units.inchesToMeters(32);
    public static final double netHeight = Units.inchesToMeters(53);
    public static final double processorHeight = 0.0;
    public static final double groundAlgaeHeight = 0.0;

    // sim settings
    public static final DCMotor gearbox = DCMotor.getKrakenX60(2);
    public static final double massKg = 18.1437 / 2.0;
    public static final double drumRadiusMeters = 0.03316409 / 2.0 * 1.06601466993;
}
