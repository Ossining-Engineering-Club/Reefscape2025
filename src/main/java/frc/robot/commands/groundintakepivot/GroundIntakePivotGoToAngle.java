package frc.robot.commands.groundintakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;

public class GroundIntakePivotGoToAngle extends Command {
    private final GroundIntakePivot groundIntakePivot;
    private final double angleSetpoint;

    public GroundIntakePivotGoToAngle(GroundIntakePivot groundIntakePivot, double angleSetpint) {
        this.groundIntakePivot = groundIntakePivot;
        this.angleSetpoint = angleSetpint;

        addRequirements(groundIntakePivot);
    }

    @Override
    public void execute() {
        groundIntakePivot.runSetpoint(angleSetpoint);
    }
    
    @Override
    public void end(boolean interrupted) {
        groundIntakePivot.stop();
    }

    @Override
    public boolean isFinished() {
        return groundIntakePivot.atSetpoint();
    }
}
