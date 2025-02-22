package frc.robot.subsystems.algaeclaw;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.algaeclaw.AlgaeClawConstants.holdingVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  /** Stops motor or runs holding voltage */
  public void stopMotor() {
    state = AlgaeClawState.STOPPED;
    if (hasAlgae()) io.setVoltage(holdingVoltage);
    else io.setVoltage(0.0);
  }

  /** Gets state of AlgaeClaw breakbeam */
  public boolean hasAlgae() {
    return photoelectricSensor.isTripped();
  }

  public Command intake() {
    return Commands.runOnce(() -> this.startMotor(), this)
            .andThen(Commands.waitUntil(() -> this.hasAlgae()))
            .andThen(Commands.waitTime(Seconds.of(AlgaeClawConstants.intakeDelaySeconds)))
            .andThen(Commands.runOnce(() -> this.stopMotor(), this));
  }

  public Command release() {
    return Commands.runOnce(() -> this.reverseMotor(), this)
            .andThen(Commands.waitUntil(() -> !this.hasAlgae()))
            .andThen(Commands.waitTime(Seconds.of(AlgaeClawConstants.releaseDelaySeconds)))
            .andThen(Commands.runOnce(() -> this.stopMotor(), this));
  }
}
