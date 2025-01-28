package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final PIDController pid;
  private final ElevatorFeedforward feedforward;

  private boolean usingPID = false;
  private int ticksSinceLastPID = 1000000;

  public Elevator(ElevatorIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        pid = new PIDController(kP, 0, kD);
        feedforward = new ElevatorFeedforward(kS, kG, 0);
        break;
      case SIM:
        pid = new PIDController(simP, 0, simD);
        feedforward = new ElevatorFeedforward(simS, simG, 0);
        break;
      case REPLAY:
        pid = new PIDController(kP, 0, kD);
        feedforward = new ElevatorFeedforward(kS, kG, 0);
        break;
      default:
        pid = new PIDController(kP, 0, kD);
        feedforward = new ElevatorFeedforward(kS, kG, 0);
        break;
    }

    pid.setTolerance(pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (ticksSinceLastPID >= 2) usingPID = false;
    ticksSinceLastPID++;

    if (Constants.currentMode == Mode.SIM && !usingPID) {
      io.setVoltage(feedforward.calculate(getHeight(), 0));
    }
  }

  public double getHeight() {
    return inputs.heightMeters;
  }

  public void runSetpoint(double heightSetpoint) {
    if (heightSetpoint > maxHeight) heightSetpoint = maxHeight;
    if (heightSetpoint < minHeight) heightSetpoint = minHeight;

    io.setVoltage(
        pid.calculate(getHeight(), heightSetpoint) + feedforward.calculate(heightSetpoint));

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
}
