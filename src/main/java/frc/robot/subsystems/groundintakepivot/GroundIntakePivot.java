package frc.robot.subsystems.groundintakepivot;

import static frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class GroundIntakePivot extends SubsystemBase {
  private final GroundIntakePivotIO io;
  private final GroundIntakePivotIOInputsAutoLogged inputs =
      new GroundIntakePivotIOInputsAutoLogged();

  private final PIDController pid;
  private final ArmFeedforward feedforward;

  private boolean usingPID = false;
  private int ticksSinceLastPID = 1000000;

  public GroundIntakePivot(GroundIntakePivotIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        pid = new PIDController(kP, 0, kD);
        feedforward = new ArmFeedforward(kS, kG, 0);
        break;
      case SIM:
        pid = new PIDController(simP, 0, simD);
        feedforward = new ArmFeedforward(simS, simG, 0);
        break;
      case REPLAY:
        pid = new PIDController(kP, 0, kD);
        feedforward = new ArmFeedforward(kS, kG, 0);
        break;
      default:
        pid = new PIDController(kP, 0, kD);
        feedforward = new ArmFeedforward(kS, kG, 0);
        break;
    }

    pid.setTolerance(pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ground Intake Pivot", inputs);

    if (ticksSinceLastPID >= 2) usingPID = false;
    ticksSinceLastPID++;

    if (Constants.currentMode == Mode.SIM && !usingPID) {
      io.setVoltage(feedforward.calculate(getAngle(), 0));
    }
  }

  public double getAngle() {
    return inputs.angleRadians;
  }

  public void runSetpoint(double angleSetpoint) {
    if (angleSetpoint > maxAngle) angleSetpoint = maxAngle;
    if (angleSetpoint < minAngle) angleSetpoint = minAngle;

    io.setVoltage(
        pid.calculate(getAngle(), angleSetpoint) + feedforward.calculate(angleSetpoint, 0));

    ticksSinceLastPID = 0;
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

  public void resetSimState() {
    io.resetSimState();
  }
}
