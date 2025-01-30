package frc.robot.subsystems.algaeclaw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.breakbeam.Breakbeam;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  private final Breakbeam breakbeam;
  private final AlgaeClawIO io;
  private final AlgaeClawIOInputsAutoLogged inputs = new AlgaeClawIOInputsAutoLogged();

  /** Algae Claw construction */
  public AlgaeClaw(AlgaeClawIO io, Breakbeam breakbeam) {
    this.io = io;
    this.breakbeam = breakbeam;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae Claw", inputs);
  }

  /** Sets motor voltage to predefined voltage */
  public void startMotor() {
    io.setVoltage(AlgaeClawConstants.clawVoltage);
  }

  /** Reverses algae claw motor */
  public void reverseMotor() {
    io.setVoltage(AlgaeClawConstants.reverseVoltage);
  }

  /** Sets motor voltage to zero */
  public void stopMotor() {
    io.setVoltage(0.0);
  }

  /** Gets state of AlgaeClaw breakbeam */
  public boolean hasAlgae() {
    return breakbeam.isTripped();
  }
}
