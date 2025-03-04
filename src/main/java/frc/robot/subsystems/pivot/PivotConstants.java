package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class PivotConstants {
    // motor constants
    public static final int canId = 17;
    public static final double motorReduction = 245.33333333333 / 1.0;
    public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
    public static final double sensorMechanismRatio =
            1 / (2 * Math.PI / motorReduction); // Rotor Rotations -> Radians
    public static final boolean isInverted = false;
    public static final int currentLimit = 38;
    public static final int stallCurrentLimit = 38;
    public static final int freeCurrentLimit = 38;

    // control system
    public static final double kP = 50.0;
    public static final double kD = 0.0;
    public static final double maxVelocity = Units.degreesToRadians(180.0);
    public static final double maxAcceleration = Units.degreesToRadians(360.0);
    public static final double simP = 50.0;
    public static final double simD = 1.25;
    public static final double simMaxVelocity = Units.degreesToRadians(720.0);
    public static final double simMaxAcceleration = Units.degreesToRadians(720.0);
    public static final double pidTolerance = Units.degreesToRadians(0.5);

    // angle setpoints
    public static final double startAngle = Units.degreesToRadians(49.0);
    public static final double maxAngle = Units.degreesToRadians(49.0);
    public static final double minAngle = -3.9744;
    public static final double storedAngle = Units.degreesToRadians(-90);
    public static final double intakeCoralAngle = 0.5453;
    public static final double intakeCoralAutoAngle = 0.458;
    public static final double placeCoralAngle = -0.539;
    public static final double placeL1CoralAngle = 0.0;
    public static final double placeL4CoralAngle = -0.304;
    public static final double knockL4CoralAngle = 0.272;
    public static final double preIntakeReefAlgaeAngle = Units.degreesToRadians(-90);
    public static final double intakeReefAlgaeAngle = -2.9073;
    public static final double removeReefAlgaeAngle = -2.82 + Units.degreesToRadians(5.0);
    public static final double processorAngle = Units.degreesToRadians(47.0);
    public static final double netAngle = Units.degreesToRadians(-115.0);
    public static final double groundAlgaeAngle = -3.9744;

    // sim settings
    public static final DCMotor gearbox = DCMotor.getFalcon500(1);
    public static final double pivotMOI = 2.058176725;
}
