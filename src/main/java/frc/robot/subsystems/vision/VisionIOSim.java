package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSim;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator estimator;
  private final Supplier<Pose2d> robotPoseSupplier;

  public VisionIOSim(CameraConfig config, Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    visionSim = new VisionSystemSim(config.name());
    visionSim.addAprilTags(TAG_LAYOUT);

    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(cameraDiagonalFOV));
    cameraProp.setCalibError(0.5, 0.08);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    camera = new PhotonCamera(config.name());
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    estimator =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCam());
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    visionSim.addCamera(cameraSim, config.robotToCam());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(robotPoseSupplier.get());

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
