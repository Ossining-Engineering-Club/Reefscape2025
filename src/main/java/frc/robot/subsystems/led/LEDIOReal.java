package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOReal implements LEDIO {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView leftData;
    private final AddressableLEDBufferView rightData;

    public LEDIOReal() {
        leds = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(length);

        leftData = buffer.createView(0, 80);
        rightData = buffer.createView(81, 149).reversed();

        leds.setLength(buffer.getLength());
        // setAll(255, 0, 0);
        setPattern(LEDPattern.solid(new Color(255, 0, 0)));
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
    public void setPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    @Override
    public void setSplitPatterns(LEDPattern left, LEDPattern right) {
        left.applyTo(leftData);
        right.applyTo(rightData);
    }

    @Override
    public void periodic() {
        leds.setData(buffer);
    }
}
