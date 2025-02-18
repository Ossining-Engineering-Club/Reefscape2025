package frc.robot.subsystems.coralholder;

import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class CoralHolderIOReal implements CoralHolderIO {
  private final SparkMax sparkMax;

  public CoralHolderIOReal() {
    sparkMax = new SparkMax(canid, MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config.smartCurrentLimit(currentLimit);
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
