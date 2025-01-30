package frc.robot.subsystems.photoelectricsensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class PhotoelectricSensorIOReal implements PhotoelectricSensorIO {
  private final AnalogInput photoelectricSensor;

  public PhotoelectricSensorIOReal(int channel) {
    photoelectricSensor = new AnalogInput(channel);
  }

  @Override
  public void updateInputs(PhotoelectricSensorIOInputs inputs) {
    inputs.voltage = photoelectricSensor.getVoltage();
  }
}
