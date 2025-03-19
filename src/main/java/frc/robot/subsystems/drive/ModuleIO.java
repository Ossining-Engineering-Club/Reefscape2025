package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveStatorCurrent = 0.0;
        public double driveSupplyCurrent = 0.0;
        public double driveTemperatureCelsius = 0.0;

        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnStatorCurrent = 0.0;
        public double turnSupplyCurrent = 0.0;
        public double turnTemperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveVoltage(double voltage) {}

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnVoltage(double voltage) {}

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {}
}
