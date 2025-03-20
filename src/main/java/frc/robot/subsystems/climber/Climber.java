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
        if (inputs.winchPosition >= maxPosition && inputs.winchMotorAppliedVolts > 0) stop();
        if (inputs.winchPosition <= minPosition && inputs.winchMotorAppliedVolts < 0) stop();
    }

    public void forward() {
        if (inputs.winchPosition < maxPosition) {
            io.setWinchMotorVoltage(winchMotorForwardVoltage);
        } else stop();
    }

    public void reverse() {
        if (inputs.winchPosition > minPosition) {
            io.setWinchMotorVoltage(winchMotorReverseVoltage);
        } else stop();
    }

    public void forwardWinch() {
        if (inputs.winchPosition < maxPosition) {
            io.setWinchMotorVoltage(winchMotorForwardVoltage);
        } else stop();
    }

    public void reverseWinch() {
        if (inputs.winchPosition > minPosition) {
            io.setWinchMotorVoltage(winchMotorReverseVoltage);
        } else stop();
    }

    public void stop() {
        io.setWinchMotorVoltage(0.0);
    }

    public double getPosition() {
        return inputs.winchPosition;
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
        return Commands.runOnce(() -> forward(), this)
                .andThen(Commands.waitUntil(() -> getPosition() >= extendPosition))
                .andThen(Commands.runOnce(() -> stop(), this));
    }

    public Command retract() {
        // return Commands.runOnce(() -> reverse(), this)
        //         .andThen(Commands.waitUntil(() -> getPosition() <= retractPosition))
        //         .andThen(Commands.runOnce(() -> stop(), this));
        return Commands.runOnce(() -> {});
    }

    public Command store() {
        return Commands.runOnce(() -> reverse(), this)
                .andThen(Commands.waitUntil(() -> getPosition() <= storePosition))
                .andThen(Commands.runOnce(() -> stop(), this));
    }
}
