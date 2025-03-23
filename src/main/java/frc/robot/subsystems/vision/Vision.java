package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.CAMERA_NAMES;
import static frc.robot.subsystems.vision.VisionConstants.TAG_LAYOUT;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimate;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIO[] ios;
    private final VisionIOInputsAutoLogged[] inputs;
    private int focusTag = 0;

    public Vision(VisionIO... ios) {
        this.ios = ios;
        inputs = new VisionIOInputsAutoLogged[ios.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            // updating vision io inputs
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + CAMERA_NAMES[i], inputs[i]);
        }
        Logger.recordOutput("focus tag", getFocusTag());
        Logger.recordOutput("sees focus tag", seesFocusTag());
    }

    public PoseEstimate[] getEstimatedGlobalPoses(Pose2d robotPose) {
        List<PoseEstimate> estimates = new ArrayList<>();
        Set<Pose3d> detectedTagPoses = new HashSet<Pose3d>();
        for (int i = 0; i < ios.length; i++) {
            // adding detected tags to list to be logged
            for (int tagId : inputs[i].tagIds) {
                VisionConstants.TAG_LAYOUT
                        .getTagPose(tagId)
                        .ifPresent(
                                (tagPose) ->
                                        detectedTagPoses.add(
                                                tagPose)); // if there's a pose, add it to the list
            }

            boolean addedPose = false;
            if (inputs[i].estimateIsPresent) {
                // don't use if estimate is outside the field
                if (!(inputs[i].estimatedPose.getX() > 0.0
                        && inputs[i].estimatedPose.getX() <= FieldConstants.fieldLengthMeters
                        && inputs[i].estimatedPose.getY() > 0.0
                        && inputs[i].estimatedPose.getY() <= FieldConstants.fieldWidthMeters))
                    continue;
                // don't use if estimate is too high, or too tilted
                if (Math.abs(inputs[i].estimatedPose.getZ()) > VisionConstants.MAX_HEIGHT) continue;
                if (Math.abs(inputs[i].estimatedPose.getRotation().getX())
                        > VisionConstants.MAX_ANGLE) continue;
                if (Math.abs(inputs[i].estimatedPose.getRotation().getY())
                        > VisionConstants.MAX_ANGLE) continue;

                double translationalDelta =
                        Math.hypot(
                                inputs[i].estimatedPose.getX() - robotPose.getX(),
                                inputs[i].estimatedPose.getY() - robotPose.getY());

                Matrix<N3, N1> stddevs =
                        getEstimationStdDevs(
                                inputs[i].estimatedPose.toPose2d(),
                                inputs[i].tagIds,
                                translationalDelta);

                addedPose = true;
                Logger.recordOutput(
                        "/Vision/" + CAMERA_NAMES[i] + "/Raw Vision",
                        inputs[i].estimatedPose.toPose2d());
                Logger.recordOutput(
                        "/Vision/" + CAMERA_NAMES[i] + "/Vision Timestamp",
                        inputs[i].timestampSeconds);
                Logger.recordOutput(
                        "/Vision/" + CAMERA_NAMES[i] + "/Vision Std Dev",
                        new double[] {stddevs.get(0, 0), stddevs.get(1, 0), stddevs.get(2, 0)});

                estimates.add(
                        new PoseEstimate(
                                inputs[i].estimatedPose.toPose2d(),
                                inputs[i].timestampSeconds,
                                stddevs));
            }
            if (!addedPose) {
                Logger.recordOutput(
                        "/Vision/" + CAMERA_NAMES[i] + "/Raw Vision",
                        new Pose2d(-1000, -1000, new Rotation2d()));
            }
        }
        // logging detected tags
        Logger.recordOutput("Detected Tag Poses", detectedTagPoses.toArray(Pose3d[]::new));

        return estimates.toArray(PoseEstimate[]::new);
    }

    public Matrix<N3, N1> getEstimationStdDevs(
            Pose2d estimatedPose, int[] tagIds, double translationalDelta) {
        var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;
        for (int tagId : tagIds) {
            // if nonexistent tag, ignore
            var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tagId);
            if (tagPose.isEmpty()) continue;

            numTags++;
            avgDist +=
                    tagPose.get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;

        avgDist /= numTags;

        // decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
        // increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 7)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        // if (numTags == 1
        //         && translationalDelta > VisionConstants.MAX_SINGLE_TAG_TRANSLATIONAL_DELTA) {
        //     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        // }

        if (VisionConstants.IGNORE_YAW) estStdDevs.set(2, 0, Double.MAX_VALUE);

        return estStdDevs;
    }

    public boolean seesFocusTag() {
        for (int i = 0; i < inputs.length; i++) {
            if (inputs[i].seesFocusTag) return true;
        }
        return false;
    }

    public PoseEstimate getSpecializedRobotPose(Rotation2d robotRotation) {
        List<Pose2d> robotPoses = new ArrayList<Pose2d>();
        double averageTimestamp = 0.0;
        for (int i = 0; i < inputs.length; i++) {
            var input = inputs[i];
            if (input.seesFocusTag) {
                averageTimestamp += input.timestampSeconds;
                Translation3d robotToTag =
                        new Translation3d(
                                input.distance
                                                * Math.cos(
                                                        Units.degreesToRadians(input.pitch)
                                                                - input.robotToCam
                                                                        .getRotation()
                                                                        .getY())
                                                * Math.cos(
                                                        Units.degreesToRadians(-input.yaw)
                                                                + input.robotToCam
                                                                        .getRotation()
                                                                        .getZ())
                                        + input.robotToCam.getX(),
                                input.distance
                                                * Math.cos(
                                                        Units.degreesToRadians(input.pitch)
                                                                - input.robotToCam
                                                                        .getRotation()
                                                                        .getY())
                                                * Math.sin(
                                                        Units.degreesToRadians(-input.yaw)
                                                                + input.robotToCam
                                                                        .getRotation()
                                                                        .getZ())
                                        + input.robotToCam.getY(),
                                input.distance
                                                * Math.cos(
                                                        Units.degreesToRadians(input.pitch)
                                                                - input.robotToCam
                                                                        .getRotation()
                                                                        .getY())
                                        + input.robotToCam.getZ());
                Pose2d robotToTagFieldRelative =
                        new Pose2d(
                                robotToTag.getX() * Math.cos(robotRotation.getRadians())
                                        - robotToTag.getY() * Math.sin(robotRotation.getRadians()),
                                robotToTag.getX() * Math.sin(robotRotation.getRadians())
                                        + robotToTag.getY() * Math.cos(robotRotation.getRadians()),
                                robotRotation);
                Logger.recordOutput("/Camera" + i + "/robotToTag", robotToTag);
                Logger.recordOutput(
                        "/Camera" + i + "/robotToTagFieldRelative", robotToTagFieldRelative);
                TAG_LAYOUT
                        .getTagPose(focusTag)
                        .ifPresent(
                                (Pose3d tagPose) -> {
                                    Pose2d robotPose =
                                            new Pose2d(
                                                    tagPose.getX() - robotToTagFieldRelative.getX(),
                                                    tagPose.getY() - robotToTagFieldRelative.getY(),
                                                    robotToTagFieldRelative.getRotation());
                                    robotPoses.add(robotPose);
                                });
            }
        }
        Logger.recordOutput("specialized pose estimates", robotPoses.toArray(Pose2d[]::new));
        if (robotPoses.size() == 0)
            return new PoseEstimate(
                    new Pose2d(),
                    0.0,
                    VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
        double sumX = 0.0;
        double sumY = 0.0;
        double sumRotation = 0.0;
        for (Pose2d pose : robotPoses) {
            sumX += pose.getX();
            sumY += pose.getY();
            sumRotation += pose.getRotation().getRadians();
        }
        Pose2d averagePose =
                new Pose2d(
                        sumX / robotPoses.size(),
                        sumY / robotPoses.size(),
                        new Rotation2d(sumRotation / robotPoses.size()));
        averageTimestamp /= robotPoses.size();
        Logger.recordOutput("average specialized pose estimate", averagePose);
        return new PoseEstimate(averagePose, averageTimestamp, VecBuilder.fill(0, 0, 0));
    }

    public void setFocusTag(int tag) {
        focusTag = tag;
        for (int i = 0; i < ios.length; i++) {
            ios[i].setFocusTag(tag);
        }
    }

    public int getFocusTag() {
        return focusTag;
    }
}
