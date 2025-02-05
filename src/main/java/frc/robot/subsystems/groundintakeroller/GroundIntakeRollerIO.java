package frc.robot.subsystems.groundintakeroller;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeRollerIO {
  @AutoLog
  public static class GroundIntakeRollerIOInputs {
    public double appliedVolts;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GroundIntakeRollerIOInputs inputs) {}

  /** Sets voltage of motor */
  public default void setVoltage(double voltage) {}
}
