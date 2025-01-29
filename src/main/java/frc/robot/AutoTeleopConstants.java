package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

public final class AutoTeleopConstants {
  public static record AlignmentConfig(String pathName, int button) {}
  ;

  public static final AlignmentConfig[] reefAlignmentConfigs =
      new AlignmentConfig[] {
        new AlignmentConfig("A", 11),
        new AlignmentConfig("B", 8),
        new AlignmentConfig("K", 5),
        new AlignmentConfig("L", 7),
      };

  public static final AlignmentConfig[] coralStationAlignmentConfigs =
      new AlignmentConfig[] {
        new AlignmentConfig("Coral Station", 6)
      };
  
  public static final AlignmentConfig processorAlignmentConfig = 
      new AlignmentConfig("Processor", 0);

  public static final PathConstraints alignmentConstraints =
      new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
}
