package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    public static final int chainMotorCanId = 14;
    public static final int winchMotorCanId = 15;

    public static final double startAngle = Units.degreesToRadians(-19.0);
    public static final boolean isInverted = false;

    public static final double chainMotorReduction = 36.0 / 15.0 * 64.0 / 1.0;
    public static final double winchMotorReduction = 64.0 / 1.0;
    public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    public static final int chainMotorStallCurrentLimit = 38;
    public static final int chainMotorFreeCurrentLimit = 38;
    public static final int winchMotorStallCurrentLimit = 38;
    public static final int winchMotorFreeCurrentLimit = 38;

    public static final double chainMotorForwardVoltage = 2.74;
    public static final double winchMotorForwardVoltage = 12.0;

    public static final double chainMotorReverseVoltage = -2.74;
    public static final double winchMotorReverseVoltage = -12.0;

    public static final double minAngle = Units.degreesToRadians(-19.0);
    public static final double maxAngle = Units.degreesToRadians(175.0);

    public static final double retractAngle = Units.degreesToRadians(43.0);
    public static final double extendAngle = Units.degreesToRadians(175.0);
    public static final double storeAngle = Units.degreesToRadians(-18.0);

    /*
     * Note:
     * Starting configuration for winch:
     * 7.5 in unspooled on right
     * 6.0 in unspooled left
     */

    // winch motor curve constants
    // public static final double chainMotorVoltage = 0.5;
    // public static final double climberRadius = 0.33; // meters
    // public static final double winchRadius = 0.017 / 2.0; // meters
    // public static final double winchMaxVoltage =
    //         (chainMotorVoltage * climberRadius * winchMotorReduction)
    //                 / (winchRadius * chainMotorReduction);
    // public static final double winchOffsetX = 0.7; // meters, back positive
    // public static final double winchOffsetY = 0.1; // meters, down positive
}
