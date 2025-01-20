package frc.robot.subsystems.breakbeam;

import org.littletonrobotics.junction.AutoLog;

public interface BreakbeamIO {
  @AutoLog
  public static class BreakbeamIOInputs {
    public double breakbeamVoltage;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(BreakbeamIOInputs inputs) {}
}
