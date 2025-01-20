package frc.robot.subsystems.breakbeam;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Breakbeam extends SubsystemBase {
  private int onCount = 0;
  private final BreakbeamIO io;
  private final BreakbeamIOInputsAutoLogged inputs = new BreakbeamIOInputsAutoLogged();
  private final String id;

  public Breakbeam(BreakbeamIO io, String id) {
    this.io = io;
    this.id = id;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Breakbeam_" + id, inputs);
    if (inputs.breakbeamVoltage < BreakbeamConstants.breakbeamVoltageThreshold) onCount++;
    else onCount = 0;
  }

  public boolean isTripped() {
    return onCount >= BreakbeamConstants.samplingWindow;
  }
}
