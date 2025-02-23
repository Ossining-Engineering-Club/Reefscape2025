package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

public final class AutoTeleopConstants {
    public static record AlignmentConfig(String pathName, int button) {}
    ;

    public static record ReefAlgaeAlignmentConfig(String pathName, int button1, int button2) {}
    ;

    public static final int aButton = 11;
    public static final int bButton = 0; // 8;
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
    public static final int netButton = 8;

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

    public static enum Level {
        L1,
        L2,
        L3,
        L4,
        LOWER_ALGAE,
        UPPER_ALGAE
    }
}
