package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.ArrayList;
import java.util.List;

public class VisionConstants {
    // public static record CameraConfig(String name, Transform3d robotToCam) {}

    public static class CameraConfig implements Sendable {
        private final String name;
        private Transform3d robotToCam;

        public CameraConfig(String name, Transform3d robotToCam) {
            this.name = name;
            this.robotToCam = robotToCam;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("CameraConfig");
            builder.addDoubleProperty("x offset", this::getXOffset, this::setXOffset);
            builder.addDoubleProperty("y offset", this::getYOffset, this::setYOffset);
            builder.addDoubleProperty("z offset", this::getZOffset, this::setZOffset);
            builder.addDoubleProperty("roll offset", this::getRollOffset, this::setRollOffset);
            builder.addDoubleProperty("pitch offset", this::getPitchOffset, this::setPitchOffset);
            builder.addDoubleProperty("yaw offset", this::getYawOffset, this::setYawOffset);
        }

        public String name() {
            return name;
        }

        public Transform3d robotToCam() {
            return robotToCam;
        }

        public double getXOffset() {
            return robotToCam.getX();
        }

        public double getYOffset() {
            return robotToCam.getY();
        }

        public double getZOffset() {
            return robotToCam.getZ();
        }

        public double getRollOffset() {
            return robotToCam.getRotation().getX();
        }

        public double getPitchOffset() {
            return robotToCam.getRotation().getY();
        }

        public double getYawOffset() {
            return robotToCam.getRotation().getZ();
        }

        public void setXOffset(double x) {
            robotToCam =
                    new Transform3d(
                            x, robotToCam.getY(), robotToCam.getZ(), robotToCam.getRotation());
        }

        public void setYOffset(double y) {
            robotToCam =
                    new Transform3d(
                            robotToCam.getX(), y, robotToCam.getZ(), robotToCam.getRotation());
        }

        public void setZOffset(double z) {
            robotToCam =
                    new Transform3d(
                            robotToCam.getX(), robotToCam.getY(), z, robotToCam.getRotation());
        }

        public void setRollOffset(double roll) {
            robotToCam =
                    new Transform3d(
                            robotToCam.getTranslation(),
                            new Rotation3d(
                                    roll,
                                    robotToCam.getRotation().getY(),
                                    robotToCam.getRotation().getZ()));
        }

        public void setPitchOffset(double pitch) {
            robotToCam =
                    new Transform3d(
                            robotToCam.getTranslation(),
                            new Rotation3d(
                                    robotToCam.getRotation().getX(),
                                    pitch,
                                    robotToCam.getRotation().getZ()));
        }

        public void setYawOffset(double yaw) {
            robotToCam =
                    new Transform3d(
                            robotToCam.getTranslation(),
                            new Rotation3d(
                                    robotToCam.getRotation().getX(),
                                    robotToCam.getRotation().getY(),
                                    yaw));
        }
    }

    public static record PoseEstimate(
            Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> standardDev) {}

    public static final AprilTagFieldLayout FULL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final List<AprilTag> WHITELISTED_APRILTAGS = new ArrayList<>();
    public static final List<Integer> BLACKLISTED_IDS =
            new ArrayList<>(List.of(14, 15, 4, 5)); // blacklisting barge tags

    static {
        for (AprilTag tag : FULL_TAG_LAYOUT.getTags()) {
            if (!BLACKLISTED_IDS.contains(tag.ID)) {
                WHITELISTED_APRILTAGS.add(tag);
            }
        }
    }

    public static final AprilTagFieldLayout TAG_LAYOUT =
            new AprilTagFieldLayout(
                    WHITELISTED_APRILTAGS,
                    FULL_TAG_LAYOUT.getFieldLength(),
                    FULL_TAG_LAYOUT.getFieldWidth());

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

    // public static final CameraConfig BACK_LEFT_CAMERA_CONFIG =
    //         new CameraConfig(
    //                 "OV9782_BL",
    //                 new Transform3d(
    //                         new Translation3d(
    //                                 Units.inchesToMeters(-11.505845),
    //                                 Units.inchesToMeters(-7.254641),
    //                                 Units.inchesToMeters(34.738822 - 0.125)),
    //                         new Rotation3d(
    //                                 0,
    //                                 Units.degreesToRadians(-50.0),
    //                                 Units.degreesToRadians(-57.5))));

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

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(3.0, 3.0, 7);
    public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

    public static final boolean IGNORE_YAW = false;

    public static final double MAX_HEIGHT = 0.305;
    public static final double MAX_ANGLE = 0.3;

    public static final double MAX_SINGLE_TAG_TRANSLATIONAL_DELTA = 1.0;

    public static final double cameraDiagonalFOV = Math.hypot(1280, 800) * 70.0 / 1280.0;
}
