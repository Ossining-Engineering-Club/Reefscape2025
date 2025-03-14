package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOReal implements LEDIO {
    private final AddressableLED leds1;
    // private final AddressableLED leds2;
    private AddressableLEDBuffer buffer;

    public LEDIOReal() {
        leds1 = new AddressableLED(pwmPort1);
        // leds2 = new AddressableLED(pwmPort2);
        buffer = new AddressableLEDBuffer(length);

        leds1.setLength(buffer.getLength());
        // setAll(255, 0, 0);
        setPattern(LEDPattern.solid(new Color(255, 0, 0)));
    }

    @Override
    public void start() {
        leds1.start();
        // leds2.start();
    }

    @Override
    public void setAll(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            buffer.setRGB(i, r, g, b);
        }
        leds1.setData(buffer);
        // leds2.setData(buffer);
    }

    @Override
    public void setPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    @Override
    public void periodic() {
        leds1.setData(buffer);
        // leds2.setData(buffer);
    }
}
