package frc.robot.subsystems.algaeclaw;

import static frc.robot.subsystems.algaeclaw.AlgaeClawConstants.holdingVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensor;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIO;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  private static enum AlgaeClawState {
    FORWARD,
    STOPPED,
    REVERSE
  }

  private final PhotoelectricSensor photoelectricSensor;
  private final AlgaeClawIO io;
  private final AlgaeClawIOInputsAutoLogged inputs = new AlgaeClawIOInputsAutoLogged();

  private AlgaeClawState state;

  /** Algae Claw construction */
  public AlgaeClaw(AlgaeClawIO io, PhotoelectricSensorIO photoelectricSensorIO) {
    this.io = io;
    this.photoelectricSensor =
        new PhotoelectricSensor(photoelectricSensorIO, AlgaeClawConstants.algaeClawPEID);
    state = AlgaeClawState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae Claw", inputs);

    Logger.recordOutput("has algae", hasAlgae());

    if (state == AlgaeClawState.STOPPED && hasAlgae()) io.setVoltage(holdingVoltage);
  }

  /** Sets motor voltage to predefined voltage */
  public void startMotor() {
    state = AlgaeClawState.FORWARD;
    io.setVoltage(AlgaeClawConstants.clawVoltage);
  }

  /** Reverses algae claw motor */
  public void reverseMotor() {
    state = AlgaeClawState.REVERSE;
    io.setVoltage(AlgaeClawConstants.reverseVoltage);
  }

  /** Sets motor voltage to zero */
  public void stopMotor() {
    state = AlgaeClawState.STOPPED;
    if (hasAlgae()) io.setVoltage(holdingVoltage);
    else io.setVoltage(0.0);
  }

  /** Gets state of AlgaeClaw breakbeam */
  public boolean hasAlgae() {
    return photoelectricSensor.isTripped();
  }
}
