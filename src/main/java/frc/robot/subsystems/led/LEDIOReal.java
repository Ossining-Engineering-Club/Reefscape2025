package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDIOReal implements LEDIO {
    private final AddressableLED leds;

    private AddressableLEDBuffer buffer;

    public LEDIOReal() {
        leds = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(length);

        leds.setLength(buffer.getLength());
        setAll(255, 0, 0);
    }

    @Override
    public void start() {
        leds.start();
    }

    @Override
    public void setAll(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            buffer.setRGB(i, r, g, b);
        }
        leds.setData(buffer);
    }

    @Override
    public void setRainbow(int offset) {
        for (int i = 0; i < length; i++) {
            buffer.setRGB(
                    i,
                    rainbow[(i + offset) % rainbow.length].red(),
                    rainbow[(i + offset) % rainbow.length].green(),
                    rainbow[(i + offset) % rainbow.length].blue());
            leds.setData(buffer);
        }
    }

    @Override
    public void setPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    @Override
    public void periodic() {
        leds.setData(buffer);
    }
}
