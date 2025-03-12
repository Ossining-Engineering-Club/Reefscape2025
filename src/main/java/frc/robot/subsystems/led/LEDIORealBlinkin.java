package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDIORealBlinkin implements LEDIO {
    private final Spark leds;

    public LEDIORealBlinkin() {
        leds = new Spark(pwmPort);
    }

    @Override
    public void set(double value) {
        leds.set(value);
    }
}
