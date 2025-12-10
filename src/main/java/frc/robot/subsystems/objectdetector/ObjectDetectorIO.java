package frc.robot.subsystems.objectdetector;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectorIO {
    @AutoLog
    public static class ObjectDetectorIOInputs {
        // first dimension is object #, second dimension is corner #
        // yaw/pitch are in radians, x/y are in pixels
        public double[] centerYaws = {}; // yaw of object center
        public double[] centerPitches = {}; // pitch of object center
        public double[][] cornerXs = {}; // x-pixel of corners
        public double[][] cornerYs = {}; // y-pixel of corners
        public int[] cornerCounts = {}; // for each object, the number of corners
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ObjectDetectorIOInputs inputs) {}
}
