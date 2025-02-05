package frc.robot.subsystems.algaeclaw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class AlgaeClawIOReal implements AlgaeClawIO {
  private final SparkMax clawMotor;

  public AlgaeClawIOReal() {
    clawMotor = new SparkMax(AlgaeClawConstants.clawCANID, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(AlgaeClawIOInputs inputs) {
    inputs.appliedVolts = clawMotor.getAppliedOutput() * clawMotor.getBusVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    clawMotor.setVoltage(voltage);
  }
}
