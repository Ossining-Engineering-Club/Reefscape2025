package frc.robot.subsystems.breakbeam;

import edu.wpi.first.wpilibj.AnalogInput;

public class BreakbeamIOReal implements BreakbeamIO {
  private final AnalogInput breakbeam;

  public BreakbeamIOReal(int channel) {
    breakbeam = new AnalogInput(channel);
  }

  @Override
  public void updateInputs(BreakbeamIOInputs inputs) {
    inputs.breakbeamVoltage = breakbeam.getVoltage();
  }
}
