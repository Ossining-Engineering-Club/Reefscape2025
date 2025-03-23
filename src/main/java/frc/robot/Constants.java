package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.REPLAY;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    // physical robot constants
    public static final double robotWidth = 0.859; // including bumpers
    public static final double robotLength = 0.991; // including bumpers
    public static final double coralIntakeXOffset = -0.023 - 0.02;
    public static final double algaeIntakeXOffset = 0.085 - 0.02;
}
