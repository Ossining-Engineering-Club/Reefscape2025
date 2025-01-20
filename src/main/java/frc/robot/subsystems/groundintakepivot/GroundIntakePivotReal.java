package frc.robot.subsystems.groundintakepivot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GroundIntakePivotReal implements GroundIntakePivotIO {
    private final SparkMax sparkMax;

    public GroundIntakePivotReal() {
        sparkMax = new SparkMax(GroundIntakePivotConstants.canId, MotorType.kBrushless);

        var config = new SparkMaxConfig();
    }

    @Override
    public void updateInputs(GroundIntakePivotIOInputs inputs) {
        inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();

    }
}
