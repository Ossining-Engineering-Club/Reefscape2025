package frc.robot.subsystems.coralholder;

import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensor;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIO;
import org.littletonrobotics.junction.Logger;

public class CoralHolder extends SubsystemBase {
  public static enum CoralHolderState {
    FORWARD,
    STOPPED,
    REVERSE
  }

  private final CoralHolderIO io;
  private final PhotoelectricSensor photoelectricSensor;
  private final CoralHolderIOInputsAutoLogged inputs = new CoralHolderIOInputsAutoLogged();

  private CoralHolderState state;

  public CoralHolder(CoralHolderIO io, PhotoelectricSensorIO photoelectricSensorIO) {
    this.io = io;
    photoelectricSensor = new PhotoelectricSensor(photoelectricSensorIO, coralHolderPEId);
    state = CoralHolderState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Coral Holder", inputs);
  }

  public void stop() {
    state = CoralHolderState.STOPPED;
    io.setVoltage(0);
  }

  public void forward() {
    state = CoralHolderState.FORWARD;
    io.setVoltage(forwardVoltage);
  }

  public void reverse() {
    state = CoralHolderState.REVERSE;
    io.setVoltage(reverseVoltage);
  }

  public boolean hasCoral() {
    return photoelectricSensor.isTripped();
  }

  public CoralHolderState getState() {
    return state;
  }
}
