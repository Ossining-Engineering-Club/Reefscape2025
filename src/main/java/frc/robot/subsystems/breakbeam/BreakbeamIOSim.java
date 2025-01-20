package frc.robot.subsystems.breakbeam;

public class BreakbeamIOSim implements BreakbeamIO {
  public BreakbeamIOSim() {}

  @Override
  public void updateInputs(BreakbeamIOInputs inputs) {
    inputs.breakbeamVoltage = 5.0;
  }
}
