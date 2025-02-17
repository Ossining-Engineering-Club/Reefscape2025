package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorGoToHeight extends Command {
  private final Elevator elevator;
  private final double heightGoal;

  public ElevatorGoToHeight(Elevator elevator, double heightGoal) {
    this.elevator = elevator;
    this.heightGoal = heightGoal;

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.runGoal(heightGoal);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.atGoal();
  }
}
