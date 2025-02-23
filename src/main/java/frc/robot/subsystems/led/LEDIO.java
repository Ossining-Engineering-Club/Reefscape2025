package frc.robot.subsystems.led;

import frc.robot.subsystems.led.LEDConstants.*;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {
        public double volts = 0;
    }
    public default void updateInput(LEDIOInputs inputs) {}
    // public default void setData(AddressabLEDBuffer buffer) {}

    public default void start() {}

    public default void setData(AddressableLEDBuffer buffer) {}

    public default void setRGB(int i, int r, int g, int b) {}



    
}
