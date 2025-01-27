package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

public final class AutoTeleopConstants {
  public static record AlignmentConfig(String pathName, int button) {}
  ;

  public static final AlignmentConfig[] alignmentConfigs =
      new AlignmentConfig[] {
        new AlignmentConfig("A", 11),
        new AlignmentConfig("B", 8),
        new AlignmentConfig("K", 5),
        new AlignmentConfig("L", 7),
        new AlignmentConfig("Coral Station", 6),
      };

  public static final PathConstraints alignmentConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
}
