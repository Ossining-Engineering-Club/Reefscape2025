package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim;
    public double appliedVolts = 0.0;

    public ElevatorIOSim() {
        elevatorSim =
            new ElevatorSim(
                gearbox,
                motorReduction,
                massKg,
                drumRadiusMeters,
                minHeight,
                maxHeight,
                true,
                startHeight);

        elevatorSim.setState(startHeight, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(0.02);

        inputs.appliedVolts = appliedVolts;
        inputs.heightMeters = elevatorSim.getPositionMeters();
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        elevatorSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void resetSimState() {
        elevatorSim.setState(startHeight, 0);
    }
}
