package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.*;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants.PositioningConfig;
import frc.robot.commands.gamepiecemanipulation.GoToProcessingPosition;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoProcessAlgae extends SequentialCommandGroup {
    public AutoProcessAlgae(
            PositioningConfig config,
            Pivot pivot,
            Elevator elevator,
            AlgaeClaw algaeClaw,
            Drive drive,
            LED led)
            throws FileVersionException, IOException, ParseException {
        addCommands(
                Commands.runOnce(() -> led.setIsPathfinding(true)),
                new GoToProcessingPosition(pivot, elevator));
    }
}
