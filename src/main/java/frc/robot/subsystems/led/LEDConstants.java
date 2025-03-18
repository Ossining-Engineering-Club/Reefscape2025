package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class LEDConstants {
    // LEDS Constant

    public static final int pwmPort = 1;
    public static final int length = 150;
    public static final int startBrightness = 100;
    public static final Distance ledSpacing = Meters.of(1.0 / 60.0);
}
