package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOReal implements ElevatorIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;

  public ElevatorIOReal() {
    sparkMax = new SparkMax(canId, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();

    var config = new SparkMaxConfig();
    config.inverted(isInverted).idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(
            1.0 / motorReduction * 2 * Math.PI * drumRadiusMeters * encoderPositionFactor)
        .velocityConversionFactor(
            1.0 / motorReduction * 2 * Math.PI * drumRadiusMeters * encoderVelocityFactor);
    config.smartCurrentLimit(currentLimit);
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(startHeight);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.heightMeters = encoder.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    sparkMax.setVoltage(voltage);
  }
}
