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
        if (inputs.angleRadians >= maxAngle && inputs.chainMotorAppliedVolts > 0) stop();
        if (inputs.angleRadians <= minAngle && inputs.chainMotorAppliedVolts < 0) stop();
    }

    public void forward() {
        if (inputs.angleRadians < maxAngle) {
            io.setChainMotorVoltage(chainMotorForwardVoltage);
            io.setWinchMotorVoltage(winchMotorForwardVoltage);
        } else stop();
    }

    public void reverse() {
        if (inputs.angleRadians > minAngle) {
            io.setChainMotorVoltage(chainMotorReverseVoltage);
            io.setWinchMotorVoltage(winchMotorReverseVoltage);
        } else stop();
    }

    public void forwardWinch() {
        if (inputs.angleRadians < maxAngle) {
            io.setWinchMotorVoltage(winchMotorForwardVoltage);
        } else stop();
    }

    public void reverseWinch() {
        if (inputs.angleRadians > minAngle) {
            io.setWinchMotorVoltage(winchMotorReverseVoltage);
        } else stop();
    }

    public void forwardChain() {
        if (inputs.angleRadians < maxAngle) {
            io.setChainMotorVoltage(chainMotorForwardVoltage);
        } else stop();
    }

    public void reverseChain() {
        if (inputs.angleRadians > minAngle) {
            io.setChainMotorVoltage(chainMotorReverseVoltage);
        } else stop();
    }

    public void stop() {
        io.setChainMotorVoltage(0.0);
        io.setWinchMotorVoltage(0.0);
    }

    public double getAngle() {
        return inputs.angleRadians;
    }

    // public double calculateWinchVoltage(double angle) {
    //     return winchMaxVoltage
    //             * Math.cos(
    //                     Math.atan(
    //                             (climberRadius * Math.sin(angle) + winchOffsetY)
    //                                             / (-climberRadius * Math.cos(angle) +
    // winchOffsetX)
    //                                     + angle
    //                                     - Math.PI / 2.0));
    // }

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

    public Command store() {
        return Commands.runOnce(() -> reverse())
                .andThen(Commands.waitUntil(() -> getAngle() <= storeAngle))
                .andThen(Commands.runOnce(() -> stop()));
    }
}
