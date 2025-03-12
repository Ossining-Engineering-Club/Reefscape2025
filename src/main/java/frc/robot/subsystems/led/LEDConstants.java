package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class LEDConstants {
    public static record Color(int red, int green, int blue) {}

    // LEDS Constant

    public static final int pwmPort1 = 1;
    public static final int pwmPort2 = 2;
    public static final int length = 120;
    public static final int startBrightness = 100;
    public static final Color[] rainbow =
            new Color[] {
                new Color(255, 0, 0),
                new Color(255, 127, 0),
                new Color(255, 255, 0),
                new Color(127, 255, 0),
                new Color(0, 255, 0),
                new Color(0, 255, 127),
                new Color(0, 255, 255),
                new Color(0, 127, 255),
                new Color(0, 0, 255),
                new Color(127, 0, 255),
                new Color(255, 0, 255),
                new Color(255, 0, 127),
            };
    public static final int ticksPerRainbowStep = 25;
    public static final Distance ledSpacing = Meters.of(1.0/60.0);
}
