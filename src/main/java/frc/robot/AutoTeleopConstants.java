package frc.robot;

import static frc.robot.AutoTeleopConstants.sidewaysReefCoralOffset;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class AutoTeleopConstants {
    public static record AlignmentConfig(String pathName, int button) {}
    ;

    public static record ReefAlgaeAlignmentConfig(String pathName, int button1, int button2) {}
    ;

    public static record PositioningConfig(
            Position position,
            String namedCommandName,
            int button,
            double sidewaysOffset,
            double depthOffset) {}
    ;

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
        LEFT_CORAL_STATION,
        RIGHT_CORAL_STATION,
        PROCESSOR
    }

    public static final int aButton = 11;
    public static final int bButton = 8;
    public static final int cButton = 0;
    public static final int dButton = 0;
    public static final int eButton = 0;
    public static final int fButton = 0;
    public static final int gButton = 0;
    public static final int hButton = 0;
    public static final int iButton = 0;
    public static final int jButton = 0;
    public static final int kButton = 5;
    public static final int lButton = 7;

    public static final int l1Button = 2;
    public static final int l2Button = 1;
    public static final int l3Button = 3;
    public static final int l4Button = 4;

    public static final int processorButton = 0;
    public static final int netButton = 0;

    public static final AlignmentConfig[] reefCoralAlignmentConfigs =
            new AlignmentConfig[] {
                new AlignmentConfig("A", aButton),
                new AlignmentConfig("B", bButton),
                new AlignmentConfig("C", cButton),
                new AlignmentConfig("D", dButton),
                new AlignmentConfig("E", eButton),
                new AlignmentConfig("F", fButton),
                new AlignmentConfig("G", gButton),
                new AlignmentConfig("H", hButton),
                new AlignmentConfig("I", iButton),
                new AlignmentConfig("J", jButton),
                new AlignmentConfig("K", kButton),
                new AlignmentConfig("L", lButton),
            };

    public static final ReefAlgaeAlignmentConfig[] reefAlgaeAlignmentConfigs =
            new ReefAlgaeAlignmentConfig[] {
                new ReefAlgaeAlignmentConfig("AB", aButton, bButton),
                new ReefAlgaeAlignmentConfig("CD", cButton, dButton),
                new ReefAlgaeAlignmentConfig("EF", eButton, fButton),
                new ReefAlgaeAlignmentConfig("GH", gButton, hButton),
                new ReefAlgaeAlignmentConfig("IJ", iButton, jButton),
                new ReefAlgaeAlignmentConfig("KL", kButton, lButton)
            };

    public static final AlignmentConfig[] coralStationAlignmentConfigs =
            new AlignmentConfig[] {
                new AlignmentConfig("Coral Station Left", 6),
                new AlignmentConfig("Coral Station Right", 0)
            };

    public static final AlignmentConfig processorAlignmentConfig =
            new AlignmentConfig("Processor", processorButton);

    public static final PathConstraints reefCoralAlignmentConstraints =
            new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final PathConstraints reefAlgaeAlignmentConstraints =
            new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final PathConstraints coralStationAlignmentConstraints =
            new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final PathConstraints processorAlignmentConstraints =
            new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final PathConstraints netAlignmentConstraints =
            new PathConstraints(1.0, 1.0, Units.degreesToRadians(360), Units.degreesToRadians(360));

    public static final double sidewaysReefCoralOffset = Units.inchesToMeters(6.5);
    public static final double depthReefCoralOffset =
            Units.inchesToMeters(2.0) + Constants.robotWidth / 2.0;

    public static final double depthCoralStationOffset = Constants.robotWidth / 2.0;

    public static final double translationalTolerance = 0.01;
    public static final double rotationalTolerance = Units.degreesToRadians(2);

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

    public static final PositioningConfig[] coralStationPositioningConfigs =
            new PositioningConfig[] {
                new PositioningConfig(
                        Position.LEFT_CORAL_STATION,
                        "Coral Station Left",
                        6,
                        -Constants.coralIntakeXOffset,
                        depthCoralStationOffset),
                new PositioningConfig(
                        Position.RIGHT_CORAL_STATION,
                        "Coral Station Right",
                        0,
                        -Constants.coralIntakeXOffset,
                        depthCoralStationOffset),
            };

    public static int getTagIdOfPosition(Position position) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            switch (position) {
                case A:
                    return 7;
                case B:
                    return 7;
                case C:
                    return 8;
                case D:
                    return 8;
                case E:
                    return 9;
                case F:
                    return 9;
                case G:
                    return 10;
                case H:
                    return 10;
                case I:
                    return 11;
                case J:
                    return 11;
                case K:
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
                case B:
                    return 18;
                case C:
                    return 17;
                case D:
                    return 17;
                case E:
                    return 22;
                case F:
                    return 22;
                case G:
                    return 21;
                case H:
                    return 21;
                case I:
                    return 20;
                case J:
                    return 20;
                case K:
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
