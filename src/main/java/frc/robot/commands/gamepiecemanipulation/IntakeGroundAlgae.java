package frc.robot.commands.gamepiecemanipulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coralpivot.CoralPivotGoToAngle;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.commands.groundintakepivot.GroundIntakePivotGoToAngle;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotConstants;
import frc.robot.subsystems.groundintakeroller.GroundIntakeRoller;

public class IntakeGroundAlgae extends SequentialCommandGroup {
  public IntakeGroundAlgae(
      GroundIntakePivot groundIntakePivot,
      CoralPivot coralPivot,
      Elevator elevator,
      GroundIntakeRoller groundIntakeRoller,
      AlgaeClaw algaeClaw) {
    addCommands(
        Commands.runOnce(() -> groundIntakeRoller.startMotor(), groundIntakeRoller),
        Commands.runOnce(() -> algaeClaw.startMotor(), algaeClaw),
        new ParallelDeadlineGroup(
            Commands.waitUntil(() -> algaeClaw.hasAlgae()),
            new GroundIntakePivotGoToAngle(
                groundIntakePivot, GroundIntakePivotConstants.extendAngle),
            new CoralPivotGoToAngle(coralPivot, CoralPivotConstants.stowAngle),
            new ElevatorGoToHeight(elevator, ElevatorConstants.groundAlgaeHeight)),
        new WaitCommand(AlgaeClawConstants.intakeDelaySeconds),
        Commands.runOnce(() -> algaeClaw.stopMotor(), algaeClaw),
        new GroundIntakePivotGoToAngle(groundIntakePivot, GroundIntakePivotConstants.stowAngle));
  }
}
