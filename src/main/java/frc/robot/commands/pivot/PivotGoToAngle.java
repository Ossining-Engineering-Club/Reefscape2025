package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class PivotGoToAngle extends Command {
    private final Pivot pivot;
    private final double angleGoal;

    public PivotGoToAngle(Pivot pivot, double angleGoal) {
        this.pivot = pivot;
        this.angleGoal = angleGoal;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.runGoal(angleGoal);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return pivot.atGoal();
    }
}
