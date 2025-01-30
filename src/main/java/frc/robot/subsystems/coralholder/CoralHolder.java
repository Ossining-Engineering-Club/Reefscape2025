package frc.robot.subsystems.coralholder;

import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.breakbeam.*;
import frc.robot.subsystems.coralholder.CoralHolderConstants.*;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensor;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIO;

import org.littletonrobotics.junction.Logger;

public class CoralHolder extends SubsystemBase {
  private final CoralHolderIO io;
  private final PhotoelectricSensor photoelectricSensor;
  private final CoralHolderIOInputsAutoLogged inputs = new CoralHolderIOInputsAutoLogged();

  public CoralHolder(CoralHolderIO io, PhotoelectricSensorIO photoelectricSensorIO) {
    this.io = io;
    photoelectricSensor = new PhotoelectricSensor(photoelectricSensorIO, coralHolderPEId);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Coral Holder", inputs);
  }

  public void stop() {
    io.setVoltage(0);
  }

  public void forward() {
    io.setVoltage(forwardVoltage);
  }

  public void reverse() {
    io.setVoltage(reverseVoltage);
  }

  public boolean hasCoral() {
    return photoelectricSensor.isTripped();
  }
}
