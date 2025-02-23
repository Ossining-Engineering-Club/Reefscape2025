package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public void forward() {
        // if (inputs.angleRadians < maxAngle) {
        io.setChainMotorVoltage(chainMotorForwardVoltage);
        // io.setRopeMotorVoltage(ropeMotorForwardVoltage);
        // } else stop();
    }

    public void reverse() {
        // if (inputs.angleRadians > minAngle) {
        io.setChainMotorVoltage(chainMotorReverseVoltage);
        // io.setRopeMotorVoltage(ropeMotorReverseVoltage);
        // } else stop();
    }

    public void stop() {
        io.setChainMotorVoltage(0.0);
        io.setRopeMotorVoltage(0.0);
    }

    public double getAngle() {
        return inputs.angleRadians;
    }

    public Command extend() {
        return Commands.runOnce(() -> forward())
                .andThen(Commands.waitUntil(() -> getAngle() >= extendAngle))
                .andThen(Commands.runOnce(() -> stop()));
    }

    public Command retract() {
        return Commands.runOnce(() -> reverse())
                .andThen(Commands.waitUntil(() -> getAngle() <= retractAngle))
                .andThen(Commands.runOnce(() -> stop()));
    }
}
