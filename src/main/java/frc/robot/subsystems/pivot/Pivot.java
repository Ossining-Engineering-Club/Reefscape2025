package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final PIDController pid;
  // private final ArmFeedforward feedforward;

  // private boolean usingPID = false;
  // private int ticksSinceLastPID = 1000000;

  public Pivot(PivotIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        pid = new PIDController(kP, 0, kD);
        // feedforward = new ArmFeedforward(kS, kG, 0);
        break;
      case SIM:
        pid = new PIDController(simP, 0, simD);
        // feedforward = new ArmFeedforward(simS, simG, 0);
        break;
      case REPLAY:
        pid = new PIDController(kP, 0, kD);
        // feedforward = new ArmFeedforward(kS, kG, 0);
        break;
      default:
        pid = new PIDController(kP, 0, kD);
        // feedforward = new ArmFeedforward(kS, kG, 0);
        break;
    }

    pid.setTolerance(pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    Logger.recordOutput("pivot angle", getAngle());

    // if (ticksSinceLastPID >= 2) usingPID = false;
    // ticksSinceLastPID++;

    // if (Constants.currentMode == Mode.SIM && !usingPID) {
    //   io.setVoltage(feedforward.calculate(getAngle(), 0));
    // }
  }

  public double getAngle() {
    return inputs.angleRadians;
  }

  public void runSetpoint(double angleSetpoint) {
    if (angleSetpoint > maxAngle) angleSetpoint = maxAngle;
    if (angleSetpoint < minAngle) angleSetpoint = minAngle;

    io.setVoltage(
        pid.calculate(getAngle(), angleSetpoint) /* + feedforward.calculate(angleSetpoint, 0)*/);

    Logger.recordOutput("pivot setpoint", angleSetpoint);

    // ticksSinceLastPID = 0;
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
