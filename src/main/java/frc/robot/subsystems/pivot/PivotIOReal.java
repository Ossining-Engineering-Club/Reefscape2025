package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class PivotIOReal implements PivotIO {
    private final SparkMax sparkMax;
    private final RelativeEncoder encoder;

    public PivotIOReal() {
        sparkMax = new SparkMax(canId, MotorType.kBrushless);
        encoder = sparkMax.getEncoder();

        var config = new SparkMaxConfig();
        config.inverted(isInverted).idleMode(IdleMode.kBrake);
        config
                .encoder
                .positionConversionFactor(1.0 / motorReduction * encoderPositionFactor)
                .velocityConversionFactor(1.0 / motorReduction * encoderVelocityFactor);
        config.smartCurrentLimit(currentLimit);
        tryUntilOk(
                sparkMax,
                5,
                () ->
                        sparkMax.configure(
                                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        encoder.setPosition(startAngle);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
        inputs.angleRadians = encoder.getPosition();
        inputs.statorCurrent = sparkMax.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        sparkMax.setVoltage(appliedVolts);
    }
}
