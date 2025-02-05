package frc.robot.commands.groundintakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.groundintakeroller.GroundIntakeRoller;

public class GroundIntakeRollerForward extends Command {
  private final GroundIntakeRoller groundIntakeRoller;
  private final AlgaeClaw algaeClaw;

  public GroundIntakeRollerForward(GroundIntakeRoller groundIntakeRoller, AlgaeClaw algaeClaw) {
    this.groundIntakeRoller = groundIntakeRoller;
    this.algaeClaw = algaeClaw;

    addRequirements(groundIntakeRoller, algaeClaw);
  }

  @Override
  public void execute() {
    groundIntakeRoller.startMotor();
  }

  @Override
  public void end(boolean interrupted) {
    groundIntakeRoller.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if (algaeClaw.hasAlgae()) return true;
    else return false;
  }
}
