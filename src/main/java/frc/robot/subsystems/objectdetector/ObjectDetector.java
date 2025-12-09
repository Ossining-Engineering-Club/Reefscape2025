package frc.robot.subsystems.objectdetector;

import static frc.robot.subsystems.objectdetector.ObjectDetectorConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetector extends SubsystemBase {
    // Line in 3-dimensions using the standard coefficient letters of the parametric form of a 3d
    // line
    private static record Line3d(double x0, double y0, double z0, double a, double b, double c) {}

    private final ObjectDetectorIO io;
    private final ObjectDetectorIOInputsAutoLogged inputs = new ObjectDetectorIOInputsAutoLogged();
    private final CameraConfig config;
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
            coral.add(
                    new Pose3d(
                            calculateTranslation(
                                    inputs.centerYaws[i],
                                    inputs.centerPitches[i],
                                    coralPlaneHeight),
                            new Rotation3d()));
        }

        Logger.recordOutput("Detected Coral", coral.toArray(Pose3d[]::new));
    }

    /** Calculates the translation of a object on */
    public Translation3d calculateTranslation(double yaw, double pitch, double planeHeight) {
        return calculateLinePlaneIntersection(calculateLine(yaw, pitch), planeHeight);
    }

    // public Rotation3d calculateRotation(double[] x, double[] y) {
    //     // sorting corners into counterclockwise order
    //     double centerX =
    // }

    /** Calculates the corresponding 3d line given a yaw and pitch angle from the camera */
    public Line3d calculateLine(double yaw, double pitch) {
        // converting camera relative yaw/pitch into field relative yaw/pitch
        yaw +=
                config.robotToCam().getRotation().getZ()
                        + robotPoseSupplier.get().getRotation().getRadians();
        pitch += config.robotToCam().getRotation().getY();

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
}
