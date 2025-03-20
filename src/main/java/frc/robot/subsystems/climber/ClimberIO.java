package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double winchMotorAppliedVolts = 0;
        public double winchPosition = startPosition; // rotations
        public double winchMotorStatorCurrent = 0.0;
        public double winchMotorTemperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {}

    /** Sets voltage of winch motor */
    public default void setWinchMotorVoltage(double voltage) {}
}
