package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorGoToHeight extends Command {
  private final Elevator elevator;
  private final double heightSetpoint;

  public ElevatorGoToHeight(Elevator elevator, double heightSetpoint) {
    this.elevator = elevator;
    this.heightSetpoint = heightSetpoint;

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.runSetpoint(heightSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.atSetpoint();
  }
}
