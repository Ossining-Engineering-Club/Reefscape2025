package frc.robot.commands.algaeclaw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;

public class AlgaeClawForward extends Command {
    private final AlgaeClaw algaeClaw;

    public AlgaeClawForward(AlgaeClaw algaeClaw) {
        this.algaeClaw = algaeClaw;

        addRequirements(algaeClaw);
    }

    @Override
    public void execute() {
        algaeClaw.startMotor();
    }

    @Override
    public void end(boolean interrupted) {
        algaeClaw.stopMotor();
    }

    @Override
    public boolean isFinished() {
        if (algaeClaw.hasAlgae()) return true;
        else return false;
    }
}
