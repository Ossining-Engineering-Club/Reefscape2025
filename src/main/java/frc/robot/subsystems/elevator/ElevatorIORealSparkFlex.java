package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class ElevatorIORealSparkFlex implements ElevatorIO {
    private final SparkFlex sparkFlex;
    private final RelativeEncoder encoder;

    public ElevatorIORealSparkFlex() {
        sparkFlex = new SparkFlex(canId, MotorType.kBrushless);
        encoder = sparkFlex.getEncoder();

        var config = new SparkMaxConfig();
        config.inverted(isInverted).idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(
                        2.0 * 1.0 / motorReduction * 2 * Math.PI * drumRadiusMeters)
                .velocityConversionFactor(
                        2.0 * 1.0 / motorReduction * 2 * Math.PI * drumRadiusMeters / 60.0);
        config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
        tryUntilOk(
                sparkFlex,
                5,
                () ->
                        sparkFlex.configure(
                                config,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        encoder.setPosition(startHeight);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = sparkFlex.getAppliedOutput() * sparkFlex.getBusVoltage();
        inputs.heightMeters = encoder.getPosition();
        inputs.statorCurrent = sparkFlex.getOutputCurrent();
        inputs.supplyCurrent =
                inputs.statorCurrent * inputs.appliedVolts / sparkFlex.getBusVoltage();
        inputs.temperatureCelsius = sparkFlex.getMotorTemperature();
    }

    @Override
    public void setVoltage(double voltage) {
        double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        Logger.recordOutput("elevator set voltage", appliedVolts);
        sparkFlex.setVoltage(appliedVolts);
    }
}
