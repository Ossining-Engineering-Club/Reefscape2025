package frc.robot.subsystems.climber;

public class ClimberConstants {
    public static final int winchMotorCanId = 15;

    public static final double startPosition = 0;
    public static final boolean isInverted = false;

    public static final double winchMotorReduction = 64.0 / 1.0;
    public static final double encoderPositionFactor = 1.0; // Rotations -> Rotations
    public static final double encoderVelocityFactor = 1.0 / 60.0; // RPM -> Rot/Sec

    public static final int chainMotorStallCurrentLimit = 38;
    public static final int chainMotorFreeCurrentLimit = 38;
    public static final int winchMotorStallCurrentLimit = 38;
    public static final int winchMotorFreeCurrentLimit = 38;

    public static final double winchMotorForwardVoltage = 12.0;
    public static final double winchMotorReverseVoltage = -12.0;

    public static final double minPosition = -10.0;
    public static final double maxPosition = 2.677;

    public static final double retractPosition = 0.0;
    public static final double extendPosition = 2.677;
    public static final double storePosition = 0.0;

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
