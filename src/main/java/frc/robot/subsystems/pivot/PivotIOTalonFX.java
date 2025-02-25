package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class PivotIOTalonFX implements PivotIO {
    private final TalonFX talon;

    public PivotIOTalonFX() {
        talon = new TalonFX(canId);

        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted =
                isInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = sensorMechanismRatio;
        config.CurrentLimits.StatorCurrentLimit = currentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
        tryUntilOk(5, () -> talon.setPosition(startAngle, 0.25));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
        inputs.angleRadians = talon.getPosition().getValueAsDouble();
        inputs.statorCurrent = talon.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = talon.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        Logger.recordOutput("pivot set voltage", appliedVolts);
        talon.setVoltage(appliedVolts);
    }
}
