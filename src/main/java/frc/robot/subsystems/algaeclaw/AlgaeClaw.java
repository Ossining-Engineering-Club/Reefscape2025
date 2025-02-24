package frc.robot.subsystems.algaeclaw;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.algaeclaw.AlgaeClawConstants.holdingVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.gamepiecevisualizers.AlgaeVisualizer;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensor;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIO;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
    private static enum AlgaeClawState {
        FORWARD,
        STOPPED,
        REVERSE
    }

    private final PhotoelectricSensor photoelectricSensor;
    private final AlgaeClawIO io;
    private final AlgaeClawIOInputsAutoLogged inputs = new AlgaeClawIOInputsAutoLogged();

    private AlgaeClawState state;

    /** Algae Claw construction */
    public AlgaeClaw(AlgaeClawIO io, PhotoelectricSensorIO photoelectricSensorIO) {
        this.io = io;
        this.photoelectricSensor =
                new PhotoelectricSensor(photoelectricSensorIO, AlgaeClawConstants.algaeClawPEID);
        state = AlgaeClawState.STOPPED;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae Claw", inputs);

        Logger.recordOutput("has algae", hasAlgae());
        SmartDashboard.putNumber("algae claw temp C", inputs.temperatureCelsius);

        if (state == AlgaeClawState.STOPPED) {
            if (hasAlgae()) io.setVoltage(holdingVoltage);
            else io.setVoltage(0.0);
        }
    }

    /** Sets motor voltage to predefined voltage */
    public void startMotor() {
        state = AlgaeClawState.FORWARD;
        io.setVoltage(AlgaeClawConstants.clawVoltage);
    }

    /** Reverses algae claw motor */
    public void reverseMotor() {
        state = AlgaeClawState.REVERSE;
        if (Constants.currentMode == Mode.SIM) {
            AlgaeVisualizer.shootAlgae();
        }
        io.setVoltage(AlgaeClawConstants.reverseVoltage);
    }

    /** Stops motor or runs holding voltage */
    public void stopMotor() {
        state = AlgaeClawState.STOPPED;
        if (hasAlgae()) io.setVoltage(holdingVoltage);
        else io.setVoltage(0.0);
    }

    /** Gets state of AlgaeClaw breakbeam */
    public boolean hasAlgae() {
        return photoelectricSensor.isTripped();
    }

    public AlgaeClawState getState() {
        return state;
    }

    public void setState(AlgaeClawState state) {
        this.state = state;
    }

    public Command intake() {
        return Commands.either(
                Commands.runOnce(() -> this.startMotor(), this)
                        .andThen(Commands.waitUntil(() -> this.hasAlgae()))
                        .andThen(
                                Commands.waitTime(
                                        Seconds.of(AlgaeClawConstants.intakeDelaySeconds)))
                        .andThen(Commands.runOnce(() -> this.stopMotor(), this)),
                Commands.runOnce(() -> {}),
                () -> !hasAlgae());
    }

    public Command release() {
        return Commands.runOnce(() -> this.reverseMotor(), this)
                .andThen(Commands.waitUntil(() -> !this.hasAlgae()))
                .andThen(Commands.waitTime(Seconds.of(AlgaeClawConstants.releaseDelaySeconds)))
                .andThen(Commands.runOnce(() -> this.stopMotor(), this));
    }
}
