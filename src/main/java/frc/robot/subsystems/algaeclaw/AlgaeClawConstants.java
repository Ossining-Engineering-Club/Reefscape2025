package frc.robot.subsystems.algaeclaw;

public class AlgaeClawConstants {

  // CAN ID
  public static int clawCANID = 1;

  // Constant Voltages
  public static double clawVoltage = 6.0;
  public static double reverseVoltage = -4.0;

  // Breakbeam IDs
  public static final String algaeClawPEID = "Algae Claw";
  public static final int algaeClawBBChannel = 0;

  public static final double intakeDelaySeconds = 0.5;
  public static final double releaseDelaySeconds = 0.5;

  // Motor constants
  public static final double currentLimit = 30;
}
