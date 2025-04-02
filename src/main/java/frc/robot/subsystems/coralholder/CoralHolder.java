package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoTeleopConstants.Level;
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
            CoralVisualizer.shootCoral(false);
        }
        io.setVoltage(reverseVoltage);
    }

    public void reverseL1() {
        state = CoralHolderState.REVERSE;
        if (Constants.currentMode == Mode.SIM) {
            CoralVisualizer.shootCoral(true);
        }
        io.setVoltage(reverseVoltageL1);
    }

    public void reverseL23() {
        state = CoralHolderState.REVERSE;
        if (Constants.currentMode == Mode.SIM) {
            CoralVisualizer.shootCoral(false);
        }
        io.setVoltage(reverseVoltageL23);
    }

    public void reverseL4Auto() {
        state = CoralHolderState.REVERSE;
        if (Constants.currentMode == Mode.SIM) {
            CoralVisualizer.shootCoral(false);
        }
        io.setVoltage(reverseVoltageL4Auto);
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

    public Command release(Level level) {
        if (level == Level.L1) {
            return Commands.runOnce(() -> this.reverseL1(), this)
                    .andThen(Commands.waitUntil(() -> !this.hasCoral()))
                    .andThen(
                            Commands.waitTime(Seconds.of(CoralHolderConstants.releaseDelaySeconds)))
                    .andThen(Commands.runOnce(() -> this.stop(), this));
        } else if (level == Level.L2 || level == Level.L3) {
            return Commands.runOnce(() -> this.reverseL23(), this)
                    .andThen(Commands.waitUntil(() -> !this.hasCoral()))
                    .andThen(
                            Commands.waitTime(Seconds.of(CoralHolderConstants.releaseDelaySeconds)))
                    .andThen(Commands.runOnce(() -> this.stop(), this));
        } else {
            return Commands.runOnce(() -> this.reverse(), this)
                    .andThen(Commands.waitUntil(() -> !this.hasCoral()))
                    .andThen(
                            Commands.waitTime(Seconds.of(CoralHolderConstants.releaseDelaySeconds)))
                    .andThen(Commands.runOnce(() -> this.stop(), this));
        }
    }

    public Command releaseAuto(Level level) {
        if (level == Level.L1) {
            return Commands.runOnce(() -> this.reverseL1(), this)
                    .andThen(Commands.waitUntil(() -> !this.hasCoral()))
                    .andThen(
                            Commands.waitTime(Seconds.of(CoralHolderConstants.releaseDelaySeconds)))
                    .andThen(Commands.runOnce(() -> this.stop(), this));
        } else if (level == Level.L2 || level == Level.L3) {
            return Commands.runOnce(() -> this.reverseL23(), this)
                    .andThen(Commands.waitUntil(() -> !this.hasCoral()))
                    .andThen(
                            Commands.waitTime(Seconds.of(CoralHolderConstants.releaseDelaySeconds)))
                    .andThen(Commands.runOnce(() -> this.stop(), this));
        } else {
            return Commands.runOnce(() -> this.reverseL4Auto(), this)
                    .andThen(Commands.waitUntil(() -> !this.hasCoral()))
                    .andThen(
                            Commands.waitTime(Seconds.of(CoralHolderConstants.releaseDelaySeconds)))
                    .andThen(Commands.runOnce(() -> this.stop(), this));
        }
    }
}
