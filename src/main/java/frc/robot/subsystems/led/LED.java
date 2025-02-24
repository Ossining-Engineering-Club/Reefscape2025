package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.coralholder.CoralHolder;

public class LED extends SubsystemBase {

    // Logging
    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

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

        this.algae = algae;
        this.coral = coral;
        this.state = LEDState.RED;

        io.start();
    }

    public void setRed() {
        io.setAll(255, 0, 0);
    }

    public void setBlue() {
        io.setAll(0, 0, 255);
    }

    public void setGreen() {
        io.setAll(0, 255, 0);
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
