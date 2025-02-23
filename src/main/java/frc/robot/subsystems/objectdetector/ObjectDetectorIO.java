package frc.robot.subsystems.objectdetector;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectorIO {
    @AutoLog
    public static class ObjectDetectorIOInputs {
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double area = 0.0;
        public boolean hasTarget = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ObjectDetectorIOInputs inputs) {}
}
