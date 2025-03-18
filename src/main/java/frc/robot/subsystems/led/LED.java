package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.led.LEDConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.coralholder.CoralHolder;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {

    // Logging
    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

    private final AlgaeClaw algae;
    private final CoralHolder coral;

    private LEDState state;
    private int tick = 0;

    private boolean isPathfinding = false;

    private static enum LEDState {
        RED,
        GREEN,
        BLUE,
        DARK_BLUE,
        PURPLE,
        RAINBOW
    }

    public LED(LEDIO io, AlgaeClaw algae, CoralHolder coral) {
        this.io = io;

        this.algae = algae;
        this.coral = coral;
        this.state = LEDState.RED;

        io.start();
    }

    public void setRed() {
        io.setPattern(LEDPattern.solid(new Color(255, 0, 0)));
    }

    public void setBlue() {
        io.setPattern(LEDPattern.solid(new Color(0, 0, 255)));
    }

    public void setGreen() {
        io.setPattern(LEDPattern.solid(new Color(0, 255, 0)));
    }

    public void setDarkBlue() {
        io.setPattern(LEDPattern.solid(new Color(0, 0, 128)));
    }

    public void setPurple() {
        io.setPattern(LEDPattern.solid(new Color(128, 0, 128)));
    }

    public void setRainbow() {
        // io.setPattern(
        //         LEDPattern.rainbow(255, 255)
        //                 .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75), ledSpacing));
        io.setSplitPatterns(
                LEDPattern.rainbow(255, 128)
                        .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75), ledSpacing),
                LEDPattern.rainbow(255, 128)
                        .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75 * 69.0 / 81.0), ledSpacing));
        // io.setSplitPatterns(
        //         LEDPattern.solid(new Color(255, 0, 0)), LEDPattern.solid(new Color(0, 255, 0)));
    }

    private void updateState() {
        if (DriverStation.isDisabled()) {
            state = LEDState.RAINBOW;
        } else if (isPathfinding) {
            state = LEDState.PURPLE;
        } else if (coral.hasCoral()) {
            state = LEDState.GREEN;
        } else if (algae.hasAlgae()) {
            state = LEDState.GREEN;
        } else {
            state = LEDState.RED;
        }
    }

    public void setIsPathfinding(boolean isPathfinding) {
        this.isPathfinding = isPathfinding;
    }

    private void updateBuffer() {
        if (state == LEDState.RAINBOW) setRainbow();
        if (state == LEDState.GREEN) setGreen();
        if (state == LEDState.RED) setRed();
        if (state == LEDState.BLUE) setBlue();
        if (state == LEDState.DARK_BLUE) setDarkBlue();
        if (state == LEDState.PURPLE) setPurple();
    }

    @Override
    public void periodic() {
        io.updateInput(inputs);

        updateState();
        updateBuffer();
        io.periodic();

        Logger.recordOutput("isPathfinding", isPathfinding);

        tick++;
    }
}
