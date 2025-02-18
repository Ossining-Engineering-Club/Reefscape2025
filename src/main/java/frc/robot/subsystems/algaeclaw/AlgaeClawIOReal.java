package frc.robot.subsystems.algaeclaw;

import static frc.robot.subsystems.algaeclaw.AlgaeClawConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeClawIOReal implements AlgaeClawIO {
  private final TalonFX clawMotor;

  public AlgaeClawIOReal() {
    clawMotor = new TalonFX(AlgaeClawConstants.clawCANID);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    clawMotor.getConfigurator().apply(config, 0.25);
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
