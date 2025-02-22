package frc.robot.subsystems.objectdetector;

import org.photonvision.PhotonCamera;

public class ObjectDetectorIOReal implements ObjectDetectorIO {
  private final PhotonCamera camera;

  public ObjectDetectorIOReal(String name) {
    camera = new PhotonCamera(name);
  }

  @Override
  public void updateInputs(ObjectDetectorIOInputs inputs) {
    // var result = camera.get
  }
}
