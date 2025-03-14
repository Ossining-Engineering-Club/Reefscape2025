package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {}

    public default void updateInput(LEDIOInputs inputs) {}

    public default void start() {}

    public default void setData(AddressableLEDBuffer buffer) {}

    public default void setAll(int r, int g, int b) {}

    public default void setPattern(LEDPattern pattern) {}

    public default void set(double value) {}

    public default void periodic() {}
}
