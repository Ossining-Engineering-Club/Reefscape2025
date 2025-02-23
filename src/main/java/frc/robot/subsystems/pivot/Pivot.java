package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final ProfiledPIDController pid;
    // private final ArmFeedforward feedforward;

    private boolean usingPID = false;
    private int ticksSinceLastPID = 1000000;

    public Pivot(PivotIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
                pid =
                        new ProfiledPIDController(
                                kP,
                                0,
                                kD,
                                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
                // feedforward = new ArmFeedforward(kS, kG, 0);
                break;
            case SIM:
                pid =
                        new ProfiledPIDController(
                                simP,
                                0,
                                simD,
                                new TrapezoidProfile.Constraints(
                                        simMaxVelocity, simMaxAcceleration));
                // feedforward = new ArmFeedforward(simS, simG, 0);
                break;
            case REPLAY:
                pid =
                        new ProfiledPIDController(
                                kP,
                                0,
                                kD,
                                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
                // feedforward = new ArmFeedforward(kS, kG, 0);
                break;
            default:
                pid =
                        new ProfiledPIDController(
                                kP,
                                0,
                                kD,
                                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
                // feedforward = new ArmFeedforward(kS, kG, 0);
                break;
        }

        io.updateInputs(inputs);
        pid.reset(inputs.angleRadians);

        pid.setTolerance(pidTolerance);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        Logger.recordOutput("pivot angle", getAngle());
        Logger.recordOutput("pivot setpoint", pid.getSetpoint().position);

        if (ticksSinceLastPID >= 2) usingPID = false;
        else usingPID = true;
        ticksSinceLastPID++;

        if (!usingPID) {
            pid.reset(getAngle());
        }

        // if (Constants.currentMode == Mode.SIM && !usingPID) {
        //   io.setVoltage(feedforward.calculate(getAngle(), 0));
        // }
    }

    public double getAngle() {
        return inputs.angleRadians;
    }

    public void runGoal(double angleGoal) {
        if (angleGoal > maxAngle) angleGoal = maxAngle;
        if (angleGoal < minAngle) angleGoal = minAngle;

        io.setVoltage(
                pid.calculate(
                        getAngle(), angleGoal) /* + feedforward.calculate(angleSetpoint, 0)*/);

        ticksSinceLastPID = 0;
    }

    public boolean atGoal() {
        return pid.atGoal();
    }

    public void stop() {
        io.setVoltage(0);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void resetSimState() {
        io.resetSimState();
    }
}
