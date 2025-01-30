package frc.robot.subsystems.groundintakepivot;

import static frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GroundIntakePivotIOSim implements GroundIntakePivotIO {
  public final SingleJointedArmSim pivotSim;
  public double appliedVolts = 0.0;

  public GroundIntakePivotIOSim() {
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
  public void updateInputs(GroundIntakePivotIOInputs inputs) {
    pivotSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.angleRadians = pivotSim.getAngleRads();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    pivotSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void resetSimState() {
    pivotSim.setState(startAngle, 0.0);
  }
}
