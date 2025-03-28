package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class AutoTeleopConstants {
    public static record PathAlignmentConfig(String pathName, int button) {}

    public static record PositioningConfig(
            Position position,
            String namedCommandName,
            Trigger trigger,
            double sidewaysOffset,
            double depthOffset) {}

    public static enum Level {
        L1,
        L2,
        L3,
        L4,
        LOWER_ALGAE,
        UPPER_ALGAE
    }

    public static enum Position {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL,
        LEFT_CORAL_STATION,
        RIGHT_CORAL_STATION,
        PROCESSOR
    }

    // Controller
    public static final CommandXboxController controller = new CommandXboxController(0);
    public static final CommandXboxController mechanismController = new CommandXboxController(1);
    public static final CommandXboxController buttonBox1 = new CommandXboxController(2);
    public static final CommandXboxController buttonBox2 = new CommandXboxController(3);

    public static final Trigger aButton = buttonBox2.button(9);
    public static final Trigger bButton = buttonBox2.button(11);
    public static final Trigger cButton = buttonBox1.button(2);
    public static final Trigger dButton = buttonBox1.button(3);
    public static final Trigger eButton = buttonBox1.button(4);
    public static final Trigger fButton = buttonBox1.button(5);
    public static final Trigger gButton = buttonBox1.button(7);
    public static final Trigger hButton = buttonBox1.button(9);
    public static final Trigger iButton = buttonBox1.button(11);
    public static final Trigger jButton = buttonBox1.button(12);
    public static final Trigger kButton = buttonBox2.button(4);
    public static final Trigger lButton = buttonBox2.button(6);

    public static final Trigger l1Button = buttonBox2.button(5);
    public static final Trigger l2Button = buttonBox2.button(3);
    public static final Trigger l3Button = buttonBox2.button(2);
    public static final Trigger l4Button = buttonBox2.button(1);

    public static final Trigger processorButton = buttonBox1.button(6);
    public static final Trigger netButton = buttonBox1.button(10);
    public static final Trigger coralStationLeftButton = buttonBox2.button(8);
    public static final Trigger coralStationRightButton = buttonBox1.button(1);
    public static final Trigger groundAlgaeButton = buttonBox2.button(7);
    public static final Trigger storedButton = buttonBox2.button(10);
    public static final Trigger cancelButton = buttonBox1.button(8);

    public static final PathConstraints reefCoralPathfindingAlignmentConstraints =
            new PathConstraints(3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(540));
    public static final PathConstraints reefCoralPIDAlignmentConstraints =
            new PathConstraints(3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(540));

    public static final PathConstraints reefAlgaePathfindingAlignmentConstraints =
            new PathConstraints(3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(540));
    public static final PathConstraints reefAlgaePIDAlignmentConstraints =
            new PathConstraints(3.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(540));

    public static final PathConstraints coralStationPathfindingAlignmentConstraints =
            new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(540));
    public static final PathConstraints coralStationPIDAlignmentConstraints =
            new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(540));

    public static final PathConstraints processorPathfindingAlignmentConstraints =
            new PathConstraints(3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(540));
    public static final PathConstraints processorPIDAlignmentConstraints =
            new PathConstraints(3.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(540));

    public static final PathConstraints netAlignmentConstraints =
            new PathConstraints(1.0, 1.0, Units.degreesToRadians(360), Units.degreesToRadians(360));

    public static final double sidewaysReefCoralOffset = Units.inchesToMeters(6.5);
    public static final double sidewaysReefCoralL1Offset = Units.inchesToMeters(1.5);
    public static final double depthReefCoralOffset =
            Units.inchesToMeters(3.0) + Constants.robotWidth / 2.0;
    public static final double depthReefCoralL23Offset =
            Units.inchesToMeters(0.5) + Constants.robotWidth / 2.0;

    public static final double sidewaysReefAlgaeOffset = Units.inchesToMeters(0);
    public static final double depthReefAlgaeOffset =
            Units.inchesToMeters(1.5) + Constants.robotWidth / 2.0;

    public static final double depthCoralStationOffset =
            Units.inchesToMeters(-1.0) + Constants.robotWidth / 2.0;

    public static final double depthProcessorOffset =
            Units.inchesToMeters(0.0) + Constants.robotWidth / 2.0;

    public static final double translationalTolerance = 0.02;
    public static final double rotationalTolerance = Units.degreesToRadians(3);

    public static final double coralStationTranslationalTolerance = 0.2;
    public static final double coralStationRotationalTolerance = Units.degreesToRadians(20);

    public static final double processorTranslationalTolerance = 0.1;
    public static final double processorRotationalTolerance = Units.degreesToRadians(10);

    public static final double reefAlgaeTranslationalTolerance = 0.05;
    public static final double reefAlgaeRotationalTolerance = Units.degreesToRadians(10);

    public static final double switchingToSpecializedTranslationalTolerance = 2.0;
    public static final double switchingToSpecializedRotationalTolerance =
            Units.degreesToRadians(10.0);

    public static final PositioningConfig[] reefCoralPositioningConfigs =
            new PositioningConfig[] {
                new PositioningConfig(
                        Position.A,
                        "A",
                        aButton,
                        sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.B,
                        "B",
                        bButton,
                        -sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.C,
                        "C",
                        cButton,
                        sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.D,
                        "D",
                        dButton,
                        -sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.E,
                        "E",
                        eButton,
                        sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.F,
                        "F",
                        fButton,
                        -sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.G,
                        "G",
                        gButton,
                        sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.H,
                        "H",
                        hButton,
                        -sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.I,
                        "I",
                        iButton,
                        sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.J,
                        "J",
                        jButton,
                        -sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.K,
                        "K",
                        kButton,
                        sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
                new PositioningConfig(
                        Position.L,
                        "L",
                        lButton,
                        -sidewaysReefCoralOffset - Constants.coralIntakeXOffset,
                        depthReefCoralOffset),
            };

    public static final PositioningConfig[] reefAlgaePositioningConfigs =
            new PositioningConfig[] {
                new PositioningConfig(
                        Position.AB,
                        "AB",
                        aButton.and(bButton),
                        sidewaysReefAlgaeOffset - Constants.algaeIntakeXOffset,
                        depthReefAlgaeOffset),
                new PositioningConfig(
                        Position.CD,
                        "CD",
                        cButton.and(dButton),
                        sidewaysReefAlgaeOffset - Constants.algaeIntakeXOffset,
                        depthReefAlgaeOffset),
                new PositioningConfig(
                        Position.EF,
                        "EF",
                        eButton.and(fButton),
                        sidewaysReefAlgaeOffset - Constants.algaeIntakeXOffset,
                        depthReefAlgaeOffset),
                new PositioningConfig(
                        Position.GH,
                        "GH",
                        gButton.and(hButton),
                        sidewaysReefAlgaeOffset - Constants.algaeIntakeXOffset,
                        depthReefAlgaeOffset),
                new PositioningConfig(
                        Position.IJ,
                        "IJ",
                        iButton.and(jButton),
                        sidewaysReefAlgaeOffset - Constants.algaeIntakeXOffset,
                        depthReefAlgaeOffset),
                new PositioningConfig(
                        Position.KL,
                        "KL",
                        kButton.and(lButton),
                        sidewaysReefAlgaeOffset - Constants.algaeIntakeXOffset,
                        depthReefAlgaeOffset),
            };

    public static final PositioningConfig[] coralStationPositioningConfigs =
            new PositioningConfig[] {
                new PositioningConfig(
                        Position.LEFT_CORAL_STATION,
                        "Coral Station Left",
                        coralStationLeftButton,
                        -Constants.coralIntakeXOffset,
                        depthCoralStationOffset),
                new PositioningConfig(
                        Position.RIGHT_CORAL_STATION,
                        "Coral Station Right",
                        coralStationRightButton,
                        -Constants.coralIntakeXOffset,
                        depthCoralStationOffset),
            };

    public static final PositioningConfig processorPositioningConfig =
            new PositioningConfig(
                    Position.PROCESSOR,
                    "Process Algae",
                    processorButton,
                    Constants.algaeIntakeXOffset,
                    depthProcessorOffset);

    public static int getTagIdOfPosition(Position position) {
        return getTagIdOfPosition(position, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public static int getTagIdOfPosition(Position position, Alliance alliance) {
        if (alliance == Alliance.Red) {
            switch (position) {
                case A:
                    return 7;
                case AB:
                    return 7;
                case B:
                    return 7;
                case C:
                    return 8;
                case CD:
                    return 8;
                case D:
                    return 8;
                case E:
                    return 9;
                case EF:
                    return 9;
                case F:
                    return 9;
                case G:
                    return 10;
                case GH:
                    return 10;
                case H:
                    return 10;
                case I:
                    return 11;
                case IJ:
                    return 11;
                case J:
                    return 11;
                case K:
                    return 6;
                case KL:
                    return 6;
                case L:
                    return 6;
                case LEFT_CORAL_STATION:
                    return 1;
                case RIGHT_CORAL_STATION:
                    return 2;
                case PROCESSOR:
                    return 3;
                default:
                    return 0;
            }
        } else {
            switch (position) {
                case A:
                    return 18;
                case AB:
                    return 18;
                case B:
                    return 18;
                case C:
                    return 17;
                case CD:
                    return 17;
                case D:
                    return 17;
                case E:
                    return 22;
                case EF:
                    return 22;
                case F:
                    return 22;
                case G:
                    return 21;
                case GH:
                    return 21;
                case H:
                    return 21;
                case I:
                    return 20;
                case IJ:
                    return 20;
                case J:
                    return 20;
                case K:
                    return 19;
                case KL:
                    return 19;
                case L:
                    return 19;
                case LEFT_CORAL_STATION:
                    return 13;
                case RIGHT_CORAL_STATION:
                    return 12;
                case PROCESSOR:
                    return 16;
                default:
                    return 0;
            }
        }
    }
}
