package frc.robot.subsystems.coralholder;

import org.littletonrobotics.junction.AutoLog;

public interface CoralHolderIO {
    @AutoLog
    public static class CoralHolderIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public default void updateInputs(CoralHolderIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
