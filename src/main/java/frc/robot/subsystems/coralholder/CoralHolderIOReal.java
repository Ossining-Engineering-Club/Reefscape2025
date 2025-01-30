package frc.robot.subsystems.coralholder;

import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralHolderIOReal implements CoralHolderIO {
  private final SparkMax sparkMax;

  public CoralHolderIOReal() {
    sparkMax = new SparkMax(canid, MotorType.kBrushless);
  }

  @Override
  public void setVoltage(double voltage) {
    sparkMax.setVoltage(voltage);
  }

  @Override
  public void updateInputs(CoralHolderIOInputs inputs) {
    inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
  }
}
