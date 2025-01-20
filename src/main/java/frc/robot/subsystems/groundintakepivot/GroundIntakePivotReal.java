package frc.robot.subsystems.groundintakepivot;

import static frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class GroundIntakePivotReal implements GroundIntakePivotIO {
    private final SparkMax sparkMax;

    public GroundIntakePivotReal() {
        sparkMax = new SparkMax(canId, MotorType.kBrushless);

        var config = new SparkMaxConfig();
        config
            .inverted(isInverted)
            .idleMode(IdleMode.kBrake);
        config
            .encoder
            .positionConversionFactor(1.0/motorReduction * encoderPositionFactor)
            .velocityConversionFactor(1.0/motorReduction * encoderVelocityFactor);
    }

    @Override
    public void updateInputs(GroundIntakePivotIOInputs inputs) {
        inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();

    }
}
