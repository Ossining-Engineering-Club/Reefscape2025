package frc.robot.subsystems.objectdetector;

import edu.wpi.first.math.util.Units;
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

            List<PhotonTrackedTarget> targets = result.getTargets();
            double[] centerYaws = new double[targets.size()];
            double[] centerPitches = new double[targets.size()];

            // second dimension for corners is set to 6 to allow
            // for a maximum of 6 corners
            double[][] cornerXs = new double[targets.size()][6];
            double[][] cornerYs = new double[targets.size()][6];
            int[] cornerCounts = new int[targets.size()];

            // iterating through targets
            for (int i = 0; i < targets.size(); i++) {
                centerYaws[i] = -Units.degreesToRadians(targets.get(i).yaw);
                centerPitches[i] = -Units.degreesToRadians(targets.get(i).pitch);
                cornerCounts[i] = targets.get(i).detectedCorners.size();

                // iterating through corners
                for (int j = 0; j < targets.get(i).detectedCorners.size(); j++) {
                    cornerXs[i][j] = targets.get(i).detectedCorners.get(j).x;
                    cornerYs[i][j] = targets.get(i).detectedCorners.get(j).y;
                }
            }

            inputs.centerYaws = centerYaws;
            inputs.centerPitches = centerPitches;
            inputs.cornerXs = cornerXs;
            inputs.cornerYs = cornerYs;
            inputs.cornerCounts = cornerCounts;
        }
    }
}
