package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOReal implements ClimberIO {
  private final SparkMax chainMotor;
  private final SparkMax ropeMotor;
  private final RelativeEncoder encoder;

  public ClimberIOReal() {
    chainMotor = new SparkMax(chainMotorCanId, MotorType.kBrushless);
    ropeMotor = new SparkMax(ropeMotorCanId, MotorType.kBrushless);
    encoder = chainMotor.getEncoder();

    var chainMotorConfig = new SparkMaxConfig();
    chainMotorConfig.inverted(isInverted).idleMode(IdleMode.kBrake);
    chainMotorConfig
        .encoder
        .positionConversionFactor(1.0 / chainMotorReduction * encoderPositionFactor)
        .velocityConversionFactor(1.0 / chainMotorReduction * encoderVelocityFactor);
    chainMotorConfig.smartCurrentLimit(chainMotorCurrentLimit);
    tryUntilOk(
        chainMotor,
        5,
        () ->
            chainMotor.configure(
                chainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder.setPosition(startAngle);

    var ropeMotorConfig = new SparkMaxConfig();
    ropeMotorConfig.idleMode(IdleMode.kBrake);
    ropeMotorConfig.smartCurrentLimit(ropeMotorCurrentLimit);
    tryUntilOk(
        ropeMotor,
        5,
        () ->
            ropeMotor.configure(
                ropeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.chainMotorAppliedVolts = chainMotor.getAppliedOutput() * chainMotor.getBusVoltage();
    inputs.ropeMotorAppliedVolts = ropeMotor.getAppliedOutput() * ropeMotor.getBusVoltage();
    inputs.angleRadians = encoder.getPosition();
  }

  @Override
  public void setChainMotorVoltage(double voltage) {
    chainMotor.setVoltage(voltage);
  }

  @Override
  public void setRopeMotorVoltage(double voltage) {
    ropeMotor.setVoltage(voltage);
  }
}
