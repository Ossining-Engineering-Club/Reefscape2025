package frc.robot.subsystems.groundintakeroller;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class GroundIntakeRollerIOReal implements GroundIntakeRollerIO {
  private final SparkMax rollerMotor;

  public GroundIntakeRollerIOReal() {
    rollerMotor = new SparkMax(GroundIntakeRollerConstants.rollerCANID, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(GroundIntakeRollerIOInputs inputs) {
    inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }
}
