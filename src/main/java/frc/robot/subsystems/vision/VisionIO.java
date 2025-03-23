package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public int[] tagIds = new int[0];
        public Pose3d estimatedPose = new Pose3d();
        public double timestampSeconds = 0.0;
        public PoseStrategy strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public boolean estimateIsPresent = false;

        // specialized pose estimation
        public double pitch = 0.0;
        public double yaw = 0.0;
        public double distance = 0.0;
        public boolean seesFocusTag = false;
        public int focusTag = 0;
        public Transform3d robotToCam = new Transform3d();

        public String cameraName = "";
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setFocusTag(int tag) {}
}
