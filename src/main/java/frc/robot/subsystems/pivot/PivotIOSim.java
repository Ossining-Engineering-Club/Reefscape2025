package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  public final DCMotorSim pivotSim;
  public double appliedVolts = 0.0;

  public PivotIOSim() {
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, pivotMOI, motorReduction), gearbox);
    // pivotSim =
    //     new SingleJointedArmSim(
    //         LinearSystemId.createSingleJointedArmSystem(gearbox, pivotMOI, motorReduction),
    //         gearbox,
    //         motorReduction,
    //         lengthMeters,
    //         minAngle,
    //         maxAngle,
    //         true,
    //         startAngle);

    pivotSim.setState(startAngle, 0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    pivotSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.angleRadians = pivotSim.getAngularPositionRad();
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
