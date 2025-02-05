package frc.robot.subsystems.photoelectricsensor;

import org.littletonrobotics.junction.AutoLog;

public interface PhotoelectricSensorIO {
  @AutoLog
  public static class PhotoelectricSensorIOInputs {
    public double voltage;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PhotoelectricSensorIOInputs inputs) {}
}
