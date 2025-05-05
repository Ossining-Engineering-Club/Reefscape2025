package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.*;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTeleopConstants.Level;
import frc.robot.AutoTeleopConstants.PositioningConfig;
import frc.robot.commands.gamepiecemanipulation.GoToPlacingCoralPosition;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoPlaceCoral extends SequentialCommandGroup {
    public AutoPlaceCoral(
            PositioningConfig config,
            Level level,
            Pivot pivot,
            Elevator elevator,
            CoralHolder coralHolder,
            Drive drive,
            Vision vision,
            LED led)
            throws FileVersionException, IOException, ParseException {
        double height =
                switch (level) {
                    case L1 -> ElevatorConstants.l1Height;
                    case L2 -> ElevatorConstants.l2Height;
                    case L3 -> ElevatorConstants.l3Height;
                    case L4 -> ElevatorConstants.l4Height;
                    default -> 0.0;
                };

        addCommands(
                Commands.runOnce(() -> led.setIsPathfinding(true)),
                new GoToPlacingCoralPosition(height, level, pivot, elevator));
    }
}
