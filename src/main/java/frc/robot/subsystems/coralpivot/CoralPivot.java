package frc.robot.subsystems.coralpivot;

import static frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralPivot extends SubsystemBase {
  private final CoralPivotIO io;
  private final CoralPivotIOInputsAutoLogged inputs = new CoralPivotIOInputsAutoLogged();

  private final PIDController pid;
  private final ArmFeedforward feedforward;

  public CoralPivot(CoralPivotIO io) {
    this.io = io;

    pid = new PIDController(kP, 0, kD);
    feedforward = new ArmFeedforward(kS, kG, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ground Intake Pivot", inputs);
  }

  public double getAngle() {
    return inputs.angleRadians;
  }

  public void runSetpoint(double angleSetpoint) {
    if (angleSetpoint > maxAngle) angleSetpoint = maxAngle;
    if (angleSetpoint < minAngle) angleSetpoint = minAngle;

    io.setVoltage(
        pid.calculate(getAngle(), angleSetpoint) + feedforward.calculate(angleSetpoint, 0));
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public void stop() {
    io.setVoltage(0);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}
