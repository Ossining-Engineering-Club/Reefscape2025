package frc.robot.subsystems.objectdetector;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectorIOReal implements ObjectDetectorIO {
    private final PhotonCamera camera;

    public ObjectDetectorIOReal(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(ObjectDetectorIOInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.size() > 0) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            PhotonTrackedTarget target = result.getBestTarget();
            inputs.yaw = target.getYaw();
            inputs.pitch = target.getPitch();
            inputs.area = target.getArea();
            inputs.hasTarget = true;
        } else {
            inputs.hasTarget = false;
        }
    }
}
