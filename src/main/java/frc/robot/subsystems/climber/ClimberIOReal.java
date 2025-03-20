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
    private final SparkMax winchMotor;
    private final RelativeEncoder encoder;

    public ClimberIOReal() {
        winchMotor = new SparkMax(winchMotorCanId, MotorType.kBrushless);
        encoder = winchMotor.getEncoder();

        var winchMotorConfig = new SparkMaxConfig();
        winchMotorConfig.inverted(isInverted).idleMode(IdleMode.kBrake);
        winchMotorConfig
                .encoder
                .positionConversionFactor(1.0 / winchMotorReduction * encoderPositionFactor)
                .velocityConversionFactor(1.0 / winchMotorReduction * encoderVelocityFactor);
        winchMotorConfig.smartCurrentLimit(winchMotorStallCurrentLimit, winchMotorFreeCurrentLimit);
        tryUntilOk(
                winchMotor,
                5,
                () ->
                        winchMotor.configure(
                                winchMotorConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        encoder.setPosition(startPosition);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.winchMotorAppliedVolts = winchMotor.getAppliedOutput() * winchMotor.getBusVoltage();
        inputs.winchPosition = encoder.getPosition();
        inputs.winchMotorStatorCurrent = winchMotor.getOutputCurrent();
        inputs.winchMotorTemperatureCelsius = winchMotor.getMotorTemperature();
    }

    @Override
    public void setWinchMotorVoltage(double voltage) {
        winchMotor.setVoltage(voltage);
    }
}
