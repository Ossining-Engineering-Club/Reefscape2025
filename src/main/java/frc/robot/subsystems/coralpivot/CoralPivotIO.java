package frc.robot.subsystems.coralpivot;

import org.littletonrobotics.junction.AutoLog;

public interface CoralPivotIO {
  @AutoLog
  public static class CoralPivotIOInputs {
    public double appliedVolts = 0.0;
    public double angleRadians = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CoralPivotIOInputs inputs) {}

  /** Sets voltage of motor */
  public default void setVoltage(double voltage) {}
}
