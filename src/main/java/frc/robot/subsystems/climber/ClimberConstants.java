package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    public static final int chainMotorCanId = 14;
    public static final int winchMotorCanId = 15;

    public static final double startAngle = Units.degreesToRadians(0.0);
    public static final boolean isInverted = false;

    public static final double chainMotorReduction = 50.0 / 1.0;
    public static final double winchMotorReduction = 50.0 / 1.0;
    public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    public static final int chainMotorStallCurrentLimit = 30;
    public static final int chainMotorFreeCurrentLimit = 30;
    public static final int winchMotorStallCurrentLimit = 30;
    public static final int winchMotorFreeCurrentLimit = 30;

    public static final double chainMotorForwardVoltage = 1.0;
    public static final double winchMotorForwardVoltage = 4.0;

    public static final double chainMotorReverseVoltage = -1.0;
    public static final double winchMotorReverseVoltage = -4.0;

    public static final double minAngle = Units.degreesToRadians(0.0);
    public static final double maxAngle = Units.degreesToRadians(180.0);

    public static final double retractAngle = Units.degreesToRadians(0.0);
    public static final double extendAngle = Units.degreesToRadians(180.0);

    // winch motor curve constants
    public static final double chainMotorVoltage = 0.5;
    public static final double climberRadius = 0.5; // meters
    public static final double winchRadius = 0.01; // meters
    public static final double winchMaxVoltage =
            (chainMotorVoltage * climberRadius * winchMotorReduction)
                    / (winchRadius * chainMotorReduction);
    public static final double winchOffsetX = 0.7; // meters, back positive
    public static final double winchOffsetY = 0.1; // meters, down positive
}
