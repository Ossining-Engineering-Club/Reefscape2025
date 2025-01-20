package frc.robot.subsystems.groundintakepivot;

import static frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GroundIntakePivotIOReal implements GroundIntakePivotIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;

  public GroundIntakePivotIOReal() {
    sparkMax = new SparkMax(canId, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();

    var config = new SparkMaxConfig();
    config.inverted(isInverted).idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(1.0 / motorReduction * encoderPositionFactor)
        .velocityConversionFactor(1.0 / motorReduction * encoderVelocityFactor);
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(startAngle);
  }

  @Override
  public void updateInputs(GroundIntakePivotIOInputs inputs) {
    inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.angleRadians = encoder.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    sparkMax.setVoltage(voltage);
  }
}
