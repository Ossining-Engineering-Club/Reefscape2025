package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final ProfiledPIDController pid;
  private final ElevatorFeedforward feedforward;

  private boolean usingPID = false;
  private int ticksSinceLastPID = 1000000;

  public Elevator(ElevatorIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        pid =
            new ProfiledPIDController(
                kP, 0, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        feedforward = new ElevatorFeedforward(kS, kG, 0);
        break;
      case SIM:
        pid =
            new ProfiledPIDController(
                simP,
                0,
                simD,
                new TrapezoidProfile.Constraints(simMaxVelocity, simMaxAcceleration));
        feedforward = new ElevatorFeedforward(simS, simG, 0);
        break;
      case REPLAY:
        pid =
            new ProfiledPIDController(
                kP, 0, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        feedforward = new ElevatorFeedforward(kS, kG, 0);
        break;
      default:
        pid =
            new ProfiledPIDController(
                kP, 0, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        feedforward = new ElevatorFeedforward(kS, kG, 0);
        break;
    }

    io.updateInputs(inputs);
    pid.reset(inputs.heightMeters);

    pid.setTolerance(pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("elevator height", getHeight());
    Logger.recordOutput("elevator setpoint", pid.getSetpoint().position);

    if (ticksSinceLastPID >= 2) usingPID = false;
    else usingPID = true;
    ticksSinceLastPID++;

    if (!usingPID) {
      pid.reset(getHeight());
    }

    if (Constants.currentMode == Mode.SIM && !usingPID) {
      io.setVoltage(feedforward.calculate(getHeight(), 0));
    }
  }

  public double getHeight() {
    return inputs.heightMeters;
  }

  public void runGoal(double heightGoal) {
    if (heightGoal > maxHeight) heightGoal = maxHeight;
    if (heightGoal < minHeight) heightGoal = minHeight;

    io.setVoltage(pid.calculate(getHeight(), heightGoal) + feedforward.calculate(heightGoal));

    ticksSinceLastPID = 0;
  }

  public boolean atGoal() {
    return pid.atGoal();
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
