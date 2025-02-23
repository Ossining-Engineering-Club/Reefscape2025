package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensor;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIO;
import org.littletonrobotics.junction.Logger;

public class CoralHolder extends SubsystemBase {
    public static enum CoralHolderState {
        FORWARD,
        STOPPED,
        REVERSE
    }

    private final CoralHolderIO io;
    private final PhotoelectricSensor photoelectricSensor;
    private final CoralHolderIOInputsAutoLogged inputs = new CoralHolderIOInputsAutoLogged();

    private CoralHolderState state;

    public CoralHolder(CoralHolderIO io, PhotoelectricSensorIO photoelectricSensorIO) {
        this.io = io;
        photoelectricSensor = new PhotoelectricSensor(photoelectricSensorIO, coralHolderPEId);
        state = CoralHolderState.STOPPED;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Holder", inputs);

        Logger.recordOutput("has coral", hasCoral());
    }

    public void stop() {
        state = CoralHolderState.STOPPED;
        io.setVoltage(0);
    }

    public void forward() {
        state = CoralHolderState.FORWARD;
        io.setVoltage(forwardVoltage);
    }

    public void reverse() {
        state = CoralHolderState.REVERSE;
        if (Constants.currentMode == Mode.SIM) {
            CoralVisualizer.shootCoral();
        }
        io.setVoltage(reverseVoltage);
    }

    public boolean hasCoral() {
        return photoelectricSensor.isTripped();
    }

    public CoralHolderState getState() {
        return state;
    }

    public void setState(CoralHolderState state) {
        this.state = state;
    }

    public Command intake() {
        return Commands.either(
                Commands.runOnce(() -> this.forward(), this)
                        .andThen(Commands.waitUntil(() -> this.hasCoral()))
                        .andThen(
                                Commands.waitTime(
                                        Seconds.of(CoralHolderConstants.intakeDelaySeconds)))
                        .andThen(Commands.runOnce(() -> this.stop(), this)),
                Commands.runOnce(() -> {}),
                () -> !hasCoral());
    }

    public Command release() {
        return Commands.runOnce(() -> this.reverse(), this)
                .andThen(Commands.waitUntil(() -> !this.hasCoral()))
                .andThen(Commands.waitTime(Seconds.of(CoralHolderConstants.releaseDelaySeconds)))
                .andThen(Commands.runOnce(() -> this.stop(), this));
    }
}
