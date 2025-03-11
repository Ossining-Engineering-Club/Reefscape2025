package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class ElevatorIORealTalonFX implements ElevatorIO {
    private final TalonFX talon;

    public ElevatorIORealTalonFX() {
        talon = new TalonFX(canId);

        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted =
                isInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = 1 / (2.0 * 1.0 / motorReduction * 2 * Math.PI * drumRadiusMeters);
        config.CurrentLimits.StatorCurrentLimit = stallCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(
                5,
                () ->
                        talon.getConfigurator().apply(config, 0.25));

        talon.setPosition(startHeight);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
        inputs.heightMeters = talon.getPosition().getValueAsDouble();
        inputs.statorCurrent = talon.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = talon.getSupplyCurrent().getValueAsDouble();
        inputs.temperatureCelsius = talon.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        Logger.recordOutput("elevator set voltage", appliedVolts);
        talon.setVoltage(appliedVolts);
    }
}
