package frc.robot.subsystems.photoelectricsensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PhotoelectricSensor extends SubsystemBase {
  private int onCount = 0;
  private final PhotoelectricSensorIO io;
  private final PhotoelectricSensorIOInputsAutoLogged inputs =
      new PhotoelectricSensorIOInputsAutoLogged();
  private final String id;

  public PhotoelectricSensor(PhotoelectricSensorIO io, String id) {
    this.io = io;
    this.id = id;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Photoelectric_" + id, inputs);
    if (inputs.voltage < PhotoelectricSensorConstants.photoelectricVoltageThreshold) onCount++;
    else onCount = 0;
  }

  public boolean isTripped() {
    return onCount >= PhotoelectricSensorConstants.samplingWindow;
  }
}
