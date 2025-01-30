package frc.robot.subsystems.coralholder;

import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.breakbeam.*;
import frc.robot.subsystems.coralholder.CoralHolderConstants.*;
import org.littletonrobotics.junction.Logger;

public class CoralHolder extends SubsystemBase {
  private final CoralHolderIO io;

  private final Breakbeam holderBeam;

  private final CoralHolderIOInputsAutoLogged inputs = new CoralHolderIOInputsAutoLogged();

  public CoralHolder(CoralHolderIO io, BreakbeamIO breakbeamio) {
    this.io = io;
    holderBeam = new Breakbeam(breakbeamio, coralHolderBBId);
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
    return holderBeam.isTripped();
  }
}
