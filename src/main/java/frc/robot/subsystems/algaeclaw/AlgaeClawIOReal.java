package frc.robot.subsystems.algaeclaw;

import static frc.robot.subsystems.algaeclaw.AlgaeClawConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeClawIOReal implements AlgaeClawIO {
  private final TalonFX clawMotor;

  public AlgaeClawIOReal() {
    clawMotor = new TalonFX(AlgaeClawConstants.clawCANID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> clawMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(AlgaeClawIOInputs inputs) {
    inputs.appliedVolts = clawMotor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = clawMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = clawMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    clawMotor.setVoltage(voltage);
  }
}
