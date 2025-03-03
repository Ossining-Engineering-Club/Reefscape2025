package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReal implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final Transform3d robotToCam;
    private final String cameraName;
    private int focusTag = 0;

    public VisionIOReal(CameraConfig config) {
        camera = new PhotonCamera(config.name());
        estimator =
                new PhotonPoseEstimator(
                        VisionConstants.TAG_LAYOUT,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        config.robotToCam());
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        robotToCam = config.robotToCam();
        cameraName = config.name();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.cameraName = cameraName;
        inputs.robotToCam = robotToCam;
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.size() > 0) {
            PhotonPipelineResult result = results.get(results.size() - 1);

            boolean resultHadFocusTag = false;
            for (var tag : result.getTargets()) {
                if (tag.fiducialId == focusTag) {
                    inputs.pitch = tag.pitch;
                    inputs.yaw = tag.yaw;
                    inputs.distance =
                            Math.hypot(
                                    tag.getBestCameraToTarget().getX(),
                                    Math.hypot(
                                            tag.getBestCameraToTarget().getY(),
                                            tag.getBestCameraToTarget().getZ()));
                    inputs.seesFocusTag = true;
                    inputs.focusTag = focusTag;
                    resultHadFocusTag = true;
                }
            }
            if (!resultHadFocusTag) inputs.seesFocusTag = false;

            // PhotonPipelineResult result = camera.getLatestResult();
            var optionalEstimate = estimator.update(result);
            if (optionalEstimate.isPresent()) {
                List<PhotonTrackedTarget> tags = result.getTargets();
                int[] tagIds = new int[tags.size()];
                for (int i = 0; i < tags.size(); i++) {
                    tagIds[i] = tags.get(i).getFiducialId();
                }
                inputs.tagIds = tagIds;

                inputs.estimatedPose = optionalEstimate.get().estimatedPose;
                inputs.timestampSeconds = optionalEstimate.get().timestampSeconds;
                inputs.strategy = optionalEstimate.get().strategy;
                inputs.estimateIsPresent = true;
            } else {
                inputs.estimateIsPresent = false;
                inputs.tagIds = new int[0];
            }
        } else {
            inputs.estimateIsPresent = false;
            inputs.tagIds = new int[0];
            inputs.seesFocusTag = false;
        }
    }

    @Override
    public void setFocusTag(int tag) {
        focusTag = tag;
    }
}
