package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReal implements VisionIO {
    public final PhotonCamera camera;
    public final PhotonPoseEstimator estimator;

    public VisionIOReal(CameraConfig config) {
        camera = new PhotonCamera(config.name());
        estimator =
                new PhotonPoseEstimator(
                        VisionConstants.TAG_LAYOUT,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        config.robotToCam());
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.size() > 0) {
            PhotonPipelineResult result = results.get(results.size() - 1);
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
        }
    }
}
