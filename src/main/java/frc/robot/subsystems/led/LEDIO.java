package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {}

    public default void updateInput(LEDIOInputs inputs) {}

    public default void start() {}

    public default void setData(AddressableLEDBuffer buffer) {}
}
