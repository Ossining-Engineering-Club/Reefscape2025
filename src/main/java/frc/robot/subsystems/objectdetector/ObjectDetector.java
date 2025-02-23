package frc.robot.subsystems.objectdetector;

import static frc.robot.subsystems.objectdetector.ObjectDetectorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ObjectDetector extends SubsystemBase {
    private final ObjectDetectorIO io;
    private final ObjectDetectorIOInputsAutoLogged inputs = new ObjectDetectorIOInputsAutoLogged();

    private int ticksSinceLastTarget = 10000;

    public ObjectDetector(ObjectDetectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae Detector", inputs);

        if (hasTarget()) ticksSinceLastTarget = 0;
    }

    public double getYaw() {
        return inputs.yaw;
    }

    public double getPitch() {
        return inputs.pitch;
    }

    public double getArea() {
        return inputs.area;
    }

    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    public boolean recentlyHadTarget() {
        return ticksSinceLastTarget <= recentlyDetectedTickLimit;
    }
}
