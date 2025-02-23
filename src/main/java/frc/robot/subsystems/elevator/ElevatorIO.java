package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double appliedVolts = 0.0;
        public double heightMeters = 0.0;
        public double statorCurrent = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Sets voltage of motor */
    public default void setVoltage(double voltage) {}

    public default void resetSimState() {}
}
