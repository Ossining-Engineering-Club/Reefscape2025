package frc.robot.commands.coralpivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralpivot.CoralPivot;

public class CoralPivotGoToAngle extends Command {
  private final CoralPivot coralPivot;
  private final double angleSetpoint;

  public CoralPivotGoToAngle(CoralPivot coralPivot, double angleSetpint) {
    this.coralPivot = coralPivot;
    this.angleSetpoint = angleSetpint;

    addRequirements(coralPivot);
  }

  @Override
  public void execute() {
    coralPivot.runSetpoint(angleSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    coralPivot.stop();
  }

  @Override
  public boolean isFinished() {
    return coralPivot.atSetpoint();
  }
}
