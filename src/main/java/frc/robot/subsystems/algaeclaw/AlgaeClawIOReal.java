package frc.robot.subsystems.algaeclaw;

import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeClawIOReal implements AlgaeClawIO {
  private final TalonFX clawMotor;

  public AlgaeClawIOReal() {
    clawMotor = new TalonFX(AlgaeClawConstants.clawCANID);
  }

  @Override
  public void updateInputs(AlgaeClawIOInputs inputs) {
    inputs.appliedVolts = clawMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    clawMotor.setVoltage(voltage);
  }
}
