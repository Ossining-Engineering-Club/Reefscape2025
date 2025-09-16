package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
    // private final DCMotorSim driveSim;
    // private final DCMotorSim turnSim;

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveSim;
    private final SimulatedMotorController.GenericMotorController turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
    private PIDController turnController = new PIDController(turnSimP, 0, turnSimD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        driveSim =
                moduleSimulation
                        .useGenericMotorControllerForDrive()
                        .withCurrentLimit(Amps.of(driveMotorStatorCurrentLimit));
        turnSim =
                moduleSimulation
                        .useGenericControllerForSteer()
                        .withCurrentLimit(Amps.of(turnMotorStatorCurrentLimit));
        // Create drive and turn sim models
        // driveSim =
        //     new DCMotorSim(
        //         LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
        //         driveGearbox);
        // turnSim =
        //     new DCMotorSim(
        //         LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
        //         turnGearbox);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(turnPIDMinInput, turnPIDMaxInput);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                    driveFFVolts
                            + driveController.calculate(
                                    moduleSimulation
                                            .getDriveWheelFinalSpeed()
                                            .in(RadiansPerSecond));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts =
                    turnController.calculate(
                            moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveSim.requestVoltage(Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)));
        turnSim.requestVoltage(Volts.of(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)));
        // driveSim.update(0.02);
        // turnSim.update(0.02);

        // Update drive inputs
        inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec =
                moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps));

        // Update turn inputs
        inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.turnVelocityRadPerSec =
                moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    }

    @Override
    public void setDriveOpenLoop(double voltage) {
        driveClosedLoop = false;
        driveAppliedVolts = voltage;
    }

    @Override
    public void setTurnOpenLoop(double voltage) {
        turnClosedLoop = false;
        turnAppliedVolts = voltage;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DriveConstants.driveSimKs * Math.signum(velocityRadPerSec) + DriveConstants.driveSimKv * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
