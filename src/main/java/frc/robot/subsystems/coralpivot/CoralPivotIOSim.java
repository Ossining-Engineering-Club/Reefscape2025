package frc.robot.subsystems.coralpivot;

import static frc.robot.subsystems.coralpivot.CoralPivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class CoralPivotIOSim implements CoralPivotIO {
  public final SingleJointedArmSim pivotSim;
  public double appliedVolts = 0.0;

  public CoralPivotIOSim() {
    pivotSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(gearbox, pivotMOI, motorReduction),
            gearbox,
            motorReduction,
            lengthMeters,
            minAngle,
            maxAngle,
            true,
            startAngle);

    pivotSim.setState(startAngle, 0.0);
  }

  @Override
  public void updateInputs(CoralPivotIOInputs inputs) {
    pivotSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.angleRadians = pivotSim.getAngleRads();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    pivotSim.setInputVoltage(appliedVolts);
  }
}
