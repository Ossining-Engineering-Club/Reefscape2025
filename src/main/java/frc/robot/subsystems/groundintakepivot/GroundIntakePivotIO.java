package frc.robot.subsystems.groundintakepivot;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakePivotIO {
  @AutoLog
  public static class GroundIntakePivotIOInputs {
    public double appliedVolts = 0.0;
    public double angleRadians = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GroundIntakePivotIOInputs inputs) {}

  /** Sets voltage of motor */
  public default void setVoltage(double voltage) {}

  public default void resetSimState() {}
}
