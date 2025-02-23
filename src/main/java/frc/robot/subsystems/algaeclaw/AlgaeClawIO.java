package frc.robot.subsystems.algaeclaw;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeClawIO {
    @AutoLog
    public static class AlgaeClawIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
        public double temperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(AlgaeClawIOInputs inputs) {}

    /** Sets voltage of motor */
    public default void setVoltage(double voltage) {}
}
