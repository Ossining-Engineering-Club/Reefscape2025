package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.coralholder.CoralHolder;

public class LED extends SubsystemBase {

    // private final String id;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    // Logging
    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

    // Patterns
    private final LEDPattern red = LEDPattern.solid(Color.kRed);
    private final LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private final LEDPattern green = LEDPattern.solid(Color.kGreen);

    // Brightness
    /*
      private final LEDPattern brightRed = red.atBrightness(Percent.of(startBrightness));
      private final LEDPattern brightBlue = blue.atBrightness(Percent.of(startBrightness));
      private final LEDPattern brightGreen = green.atBrightness(Percent.of(startBrightness));
    */

    // Passing through Algae and Claw
    private final AlgaeClaw algae;
    private final CoralHolder coral;

    private LEDState state;

    private static enum LEDState {
        RED,
        GREEN,
        BLUE
    }

    public LED(LEDIO io, AlgaeClaw algae, CoralHolder coral) {
        this.io = io;
        // this.id = id;

        this.algae = algae;
        this.coral = coral;

        this.state = LEDState.RED;

        led = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(length);

        // Set Length
        led.setLength(buffer.getLength());

        // Start Brightness
        /*
        brightRed.applyTo(buffer);
        brightBlue.applyTo(buffer);
        brightGreen.applyTo(buffer);

        */

        // Start Leds
        led.start();
    }

    /*
    public void setBrightness(int percent) {
      brightRed.atBrightness(Percent.of(percent));
      brightBlue.atBrightness(Percent.of(percent));
      brightGreen.atBrightness(Percent.of(percent));

      brightRed.applyTo(buffer);
      brightBlue.applyTo(buffer);
      brightGreen.applyTo(buffer);
    }
      */

    public void setRed() {
        red.applyTo(buffer);
        led.setData(buffer);
    }

    public void setBlue() {
        blue.applyTo(buffer);
        led.setData(buffer);
    }

    public void setGreen() {
        green.applyTo(buffer);
        led.setData(buffer);
    }

    private void updateState() {
        if (coral.hasCoral()) {
            state = LEDState.GREEN;
        } else if (algae.hasAlgae()) {
            state = LEDState.GREEN;
        } else {
            state = LEDState.RED;
        }
    }

    private void updateBuffer() {
        if (state == LEDState.GREEN) setGreen();
        if (state == LEDState.RED) setRed();
        if (state == LEDState.BLUE) setBlue();
    }

    @Override
    public void periodic() {
        io.updateInput(inputs);

        updateState();
        updateBuffer();
    }
}
