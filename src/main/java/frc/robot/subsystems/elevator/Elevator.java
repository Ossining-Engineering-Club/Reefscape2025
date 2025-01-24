package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final PIDController pid;
  private final ElevatorFeedforward feedforward;

  public Elevator(ElevatorIO io) {
    this.io = io;

    pid = new PIDController(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public void runSetpoint(double positionSetpoint) {
    if (positionSetpoint > maxPosition) positionSetpoint = maxPosition;
    if (positionSetpoint < minPosition) positionSetpoint = minPosition;

    io.setVoltage(
        pid.calculate(getPosition(), positionSetpoint) + feedforward.calculate(positionSetpoint));
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
