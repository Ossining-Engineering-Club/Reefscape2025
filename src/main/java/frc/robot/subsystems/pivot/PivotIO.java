package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double appliedVolts = 0.0;
        public double angleRadians = 0.0;
        public double statorCurrent = 0.0;
        public double temperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {}

    /** Sets voltage of motor */
    public default void setVoltage(double voltage) {}

    public default void resetSimState() {}
}
