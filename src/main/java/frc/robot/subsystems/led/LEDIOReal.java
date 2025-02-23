package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOReal implements LEDIO {
    private final AddressableLED leds;

    private AddressableLEDBuffer buffer;

    public LEDIOReal() {
        leds = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(120);
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
    }
}
