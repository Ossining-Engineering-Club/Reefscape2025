package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class PivotGoToAngle extends Command {
  private final Pivot pivot;
  private final double angleSetpoint;

  public PivotGoToAngle(Pivot pivot, double angleSetpint) {
    this.pivot = pivot;
    this.angleSetpoint = angleSetpint;

    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.runSetpoint(angleSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }

  @Override
  public boolean isFinished() {
    return pivot.atSetpoint();
  }
}
