package frc.robot.subsystems.algaeclaw;

public class AlgaeClawConstants {

    // CAN ID
    public static int clawCANID = 18;

    // Constant Voltages
    public static double reefClawVoltage = 4.0;
    public static double groundClawVoltage = 8.0;
    public static double reverseVoltage = -4.0;
    public static double holdingVoltage = 0.36;

    // Breakbeam IDs
    public static final String algaeClawPEID = "Algae Claw";
    public static final int algaeClawPEChannel = 1;

    public static final double intakeDelaySeconds = 0.25;
    public static final double releaseDelaySeconds = 0.5;

    // Motor constants
    public static final double currentLimit = 30;
}
