package frc.robot.subsystems.photoelectricsensor;

public class PhotoelectricSensorIOSim implements PhotoelectricSensorIO {
  public PhotoelectricSensorIOSim() {}

  @Override
  public void updateInputs(PhotoelectricSensorIOInputs inputs) {
    inputs.voltage = 5.0;
  }
}
