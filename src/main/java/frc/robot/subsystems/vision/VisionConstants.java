package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

public class VisionConstants {
    public static record CameraConfig(String name, Transform3d robotToCam) {}

    public static record PoseEstimate(
            Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> standardDev) {}

    public static final AprilTagFieldLayout TAG_LAYOUT = getTagLayout();
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private static AprilTagFieldLayout getTagLayout() {
        try {
            return new AprilTagFieldLayout("C:\\Users\\eric\\Downloads\\field_calibration.json");
        } catch (IOException e) {
            e.printStackTrace();
            return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        }
    }

    public static final CameraConfig FRONT_LEFT_CAMERA_CONFIG =
            new CameraConfig(
                    "OV9782_FL",
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(13.72048),
                                    Units.inchesToMeters(10.202495),
                                    Units.inchesToMeters(7.780637)),
                            new Rotation3d(
                                    0,
                                    Units.degreesToRadians(-21.875),
                                    Units.degreesToRadians(90 + 35))));

    public static final CameraConfig FRONT_RIGHT_CAMERA_CONFIG =
            new CameraConfig(
                    "OV9782_FR",
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(13.72048),
                                    Units.inchesToMeters(-10.202495),
                                    Units.inchesToMeters(7.780637)),
                            new Rotation3d(
                                    0,
                                    Units.degreesToRadians(-21.875),
                                    Units.degreesToRadians(-90 - 35))));

    public static final CameraConfig BACK_LEFT_CAMERA_CONFIG =
            new CameraConfig(
                    "OV9782_BL",
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(-13.72048),
                                    Units.inchesToMeters(10.202495),
                                    Units.inchesToMeters(7.780637)),
                            new Rotation3d(
                                    0,
                                    Units.degreesToRadians(-21.875),
                                    Units.degreesToRadians(90 - 35))));

    public static final CameraConfig BACK_RIGHT_CAMERA_CONFIG =
            new CameraConfig(
                    "OV9281_BR",
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(-13.72048),
                                    Units.inchesToMeters(-10.202495),
                                    Units.inchesToMeters(7.780637)),
                            new Rotation3d(
                                    0,
                                    Units.degreesToRadians(-21.875),
                                    Units.degreesToRadians(-90 + 35))));

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
    public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

    public static final boolean IGNORE_YAW = false;

    public static final double MAX_HEIGHT = 0.305;
    public static final double MAX_ANGLE = 0.3;

    public static final double cameraDiagonalFOV = Math.hypot(1280, 800) * 70.0 / 1280.0;
}
