package frc.robot.subsystems.objectdetector;

import static frc.robot.subsystems.objectdetector.ObjectDetectorConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetector extends SubsystemBase {
    // Line in 3-dimensions using the standard coefficient letters of the parametric form of a 3d
    // line
    private static record Line3d(double x0, double y0, double z0, double a, double b, double c) {}
    // Pixel coordinate
    private static record Pixel(double x, double y) {}

    private final ObjectDetectorIO io;
    private final ObjectDetectorIOInputsAutoLogged inputs = new ObjectDetectorIOInputsAutoLogged();
    private CameraConfig config;
    private final Supplier<Pose2d> robotPoseSupplier;

    ArrayList<Pose3d> coral = new ArrayList<>();
    ArrayList<Pose3d> algae = new ArrayList<>();

    public ObjectDetector(
            ObjectDetectorIO io, CameraConfig config, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.config = config;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Object Detector", inputs);

        coral.clear();
        algae.clear();

        // iterating through detected objects
        for (int i = 0; i < inputs.centerYaws.length; i++) {
            // adding poses of detected coral
            coral.add(
                    new Pose3d(
                            calculateTranslation(
                                            inputs.centerYaws[i],
                                            inputs.centerPitches[i],
                                            coralPlaneHeight)
                                    .plus(new Translation3d(0, 0, -0.1)),
                            new Rotation3d(0, Math.PI / 2.0, 0)));
        }

        Logger.recordOutput("Detected Coral", coral.toArray(Pose3d[]::new));
        // SmartDashboard.putData("Object Detector Config", config);
    }

    public ArrayList<Pose3d> getCoral() {
        return coral;
    }

    public Optional<Pose3d> getClosestCoralToIntake() {
        if (coral.size() == 0) return Optional.empty();
        double closestDistance = 9999999;
        int closestCoralIndex = -1;
        Translation2d intakePosFieldRelative =
                new Translation2d(
                        robotPoseSupplier.get().getX()
                                + Constants.intakePosition.getX()
                                        * Math.cos(
                                                robotPoseSupplier.get().getRotation().getRadians())
                                + Constants.intakePosition.getY()
                                        * Math.sin(
                                                robotPoseSupplier.get().getRotation().getRadians()),
                        robotPoseSupplier.get().getY()
                                + Constants.intakePosition.getX()
                                        * Math.sin(
                                                robotPoseSupplier.get().getRotation().getRadians())
                                + Constants.intakePosition.getY()
                                        * Math.cos(
                                                robotPoseSupplier
                                                        .get()
                                                        .getRotation()
                                                        .getRadians()));
        SmartDashboard.putNumber("intakePosFieldRelativeX", intakePosFieldRelative.getX());
        SmartDashboard.putNumber("intakePosFieldRelativeY", intakePosFieldRelative.getY());
        for (int i = 0; i < coral.size(); i++) {
            double distance =
                    intakePosFieldRelative.getDistance(
                            coral.get(i).getTranslation().toTranslation2d());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestCoralIndex = i;
            }
        }
        return Optional.of(coral.get(closestCoralIndex));
    }

    /** Calculates the translation of a object with its center planeHeight above the ground */
    public Translation3d calculateTranslation(double yaw, double pitch, double planeHeight) {
        return calculateLinePlaneIntersection(calculateLine(yaw, pitch), planeHeight);
    }

    /** Calculates the rotation of a object with its center planeHeight above the ground */
    public Rotation3d calculateRotation(
            double[] x, double[] y, int numCorners, double planeHeight) {
        // finding center of object by averaging corner coordinates
        double centerX = 0;
        double centerY = 0;
        for (int i = 0; i < numCorners; i++) centerX += x[i];
        for (int i = 0; i < numCorners; i++) centerY += y[i];
        centerX /= numCorners;
        centerY /= numCorners;

        // first element in pair stores the angle (relative to +x) in radians of the vector going
        // from the center the corner
        // second element in pair stores the index of the corner in the original array of
        // coordinates
        ArrayList<Pair<Double, Integer>> centerToCornerAngles = new ArrayList<>();
        for (int i = 0; i < numCorners; i++) {
            // calculating angle of vector
            // y distance is negated because y+ is down in PhotonVision
            double angle = Math.atan2(-(y[i] - centerY), x[i] - centerX);
            centerToCornerAngles.add(new Pair<Double, Integer>(angle, i));
        }
        // sorting in counterclockwise order
        centerToCornerAngles.sort((a, b) -> a.getFirst().compareTo(b.getFirst()));

        // creating a array list of corner coordinates (in pixels, (x,y) format) sorted in
        // counterclockwise order
        ArrayList<Pixel> corners = new ArrayList<>();
        for (Pair<Double, Integer> angleAndIndex : centerToCornerAngles) {
            corners.add(new Pixel(x[angleAndIndex.getSecond()], y[angleAndIndex.getSecond()]));
        }
        // from now on, each corner is identified by its index in the corners ArrayList

        // edge i represents the edge from corner i to corner i+1 (or corner 0 if i = numCorners-1)
        double[] edgeLengths = new double[numCorners];
        int longestEdge = -1;
        double longestEdgeLength = 0;
        int secondLongestEdge = -1;
        for (int i = 0; i < numCorners; i++) {
            edgeLengths[i] = calculateDistance(corners.get(i), corners.get((i + 1) % numCorners));
            // calculating longest and second longest edges
            if (edgeLengths[i] >= longestEdgeLength) {
                secondLongestEdge = longestEdge;
                longestEdge = i;
                longestEdgeLength = edgeLengths[i];
            }
        }

        // finding the midpoints between corresponding corners of the two longest edges
        // *-------------------------*
        // |                         |
        // A                         B
        // |                         |
        // *-------------------------*
        // for example, if the above was our polygon, and the top and bottom were our two
        // longest edges, then we would be finding points A and B
        Pixel point1 =
                new Pixel(
                        (corners.get(longestEdge).x
                                        + corners.get((secondLongestEdge + 1) % numCorners).x)
                                / 2.0,
                        (corners.get(longestEdge).y
                                        + corners.get((secondLongestEdge + 1) % numCorners).y)
                                / 2.0);
        Pixel point2 =
                new Pixel(
                        (corners.get((longestEdge + 1) % numCorners).x
                                        + corners.get(secondLongestEdge).x)
                                / 2.0,
                        (corners.get((longestEdge + 1) % numCorners).y
                                        + corners.get(secondLongestEdge).y)
                                / 2.0);

        // converting pixels of midpoints to yaw/pitch
        double yaw1 = (imageWidthPx / 2.0 - point1.x) / imageWidthPx * horizontalFOV;
        double pitch1 = -(imageHeightPx / 2.0 - point1.y) / imageHeightPx * verticalFOV;
        double yaw2 = (imageWidthPx / 2.0 - point2.x) / imageWidthPx * horizontalFOV;
        double pitch2 = -(imageHeightPx / 2.0 - point2.y) / imageHeightPx * verticalFOV;

        // projecting points in image onto the object center plane
        Translation3d projectedPoint1 =
                calculateLinePlaneIntersection(calculateLine(yaw1, pitch1), planeHeight);
        Translation3d projectedPoint2 =
                calculateLinePlaneIntersection(calculateLine(yaw2, pitch2), planeHeight);

        return new Rotation3d(
                0,
                0,
                Math.atan2(
                        projectedPoint1.getY() - projectedPoint2.getY(),
                        projectedPoint1.getX() - projectedPoint2.getX()));
    }

    /** Calculates the corresponding 3d line given a yaw and pitch angle from the camera */
    public Line3d calculateLine(double yaw, double pitch) {
        // converting camera relative yaw/pitch into field relative yaw/pitch
        SmartDashboard.putNumber("raw yaw", yaw);
        SmartDashboard.putNumber("raw pitch", pitch);
        yaw +=
                config.robotToCam().getRotation().getZ()
                        + robotPoseSupplier.get().getRotation().getRadians();
        pitch += config.robotToCam().getRotation().getY();
        SmartDashboard.putNumber("fr yaw", yaw);
        SmartDashboard.putNumber("fr pitch", pitch);

        return new Line3d(
                config.robotToCam().getX()
                                * Math.cos(robotPoseSupplier.get().getRotation().getRadians())
                        + config.robotToCam().getY()
                                * Math.sin(robotPoseSupplier.get().getRotation().getRadians())
                        + robotPoseSupplier.get().getX(),
                config.robotToCam().getX()
                                * Math.sin(robotPoseSupplier.get().getRotation().getRadians())
                        + config.robotToCam().getY()
                                * Math.cos(robotPoseSupplier.get().getRotation().getRadians())
                        + robotPoseSupplier.get().getY(),
                config.robotToCam().getZ(),
                Math.cos(yaw) * Math.cos(-pitch),
                Math.sin(yaw) * Math.cos(-pitch),
                Math.sin(-pitch));
    }

    /**
     * Calculates the intersection of a 3d line and a horizontal plane that is a height of
     * planeHeight above the ground
     */
    public Translation3d calculateLinePlaneIntersection(Line3d line, double planeHeight) {
        double t = (planeHeight - line.z0) / line.c;
        return new Translation3d(line.x0 + line.a * t, line.y0 + line.b * t, planeHeight);
    }

    /** Calculates distance between two pixel coordinates */
    public double calculateDistance(Pixel a, Pixel b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }
}
