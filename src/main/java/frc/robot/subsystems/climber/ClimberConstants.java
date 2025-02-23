package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    public static final int chainMotorCanId = 14;
    public static final int ropeMotorCanId = 15;

    public static final double startAngle = Units.degreesToRadians(0.0);
    public static final boolean isInverted = false;

    public static final double chainMotorReduction = 50.0 / 1.0;
    public static final double ropeMotorReduction = 50.0 / 1.0;
    public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    public static final int chainMotorCurrentLimit = 30;
    public static final int ropeMotorCurrentLimit = 30;

    public static final double chainMotorForwardVoltage = 1.0;
    public static final double ropeMotorForwardVoltage = 4.0;

    public static final double chainMotorReverseVoltage = -1.0;
    public static final double ropeMotorReverseVoltage = -4.0;

    public static final double minAngle = Units.degreesToRadians(0.0);
    public static final double maxAngle = Units.degreesToRadians(180.0);

    public static final double retractAngle = Units.degreesToRadians(0.0);
    public static final double extendAngle = Units.degreesToRadians(180.0);
}
