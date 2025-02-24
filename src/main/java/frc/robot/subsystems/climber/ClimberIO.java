package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double chainMotorAppliedVolts = 0;
        public double winchMotorAppliedVolts = 0;
        public double angleRadians = startAngle;
        public double chainMotorStatorCurrent = 0.0;
        public double winchMotorStatorCurrent = 0.0;
        public double chainMotorTemperatureCelsius = 0.0;
        public double winchMotorTemperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {}

    /** Sets voltage of chain motor */
    public default void setChainMotorVoltage(double voltage) {}

    /** Sets voltage of winch motor */
    public default void setWinchMotorVoltage(double voltage) {}
}
