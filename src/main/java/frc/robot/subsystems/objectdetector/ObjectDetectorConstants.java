package frc.robot.subsystems.objectdetector;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;

public class ObjectDetectorConstants {
    public static final CameraConfig OBJECT_CAMERA_CONFIG =
            new CameraConfig(
                    "OV9782_BL",
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(-11.505845),
                                    Units.inchesToMeters(-7.254641),
                                    Units.inchesToMeters(34.738822 - 0.125)),
                            new Rotation3d(
                                    0,
                                    Units.degreesToRadians(50.0),
                                    Units.degreesToRadians(-57.5))));

    public static final double coralPlaneHeight =
            Units.inchesToMeters(
                    2.25); // height of center of coral off the ground when laid down on the ground
}
