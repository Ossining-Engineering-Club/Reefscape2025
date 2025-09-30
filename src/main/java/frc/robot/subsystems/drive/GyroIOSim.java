package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSim;

    public GyroIOSim(GyroSimulation gyroSim) {
        this.gyroSim = gyroSim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSim.getGyroReading();
        inputs.yawVelocityRadPerSec =
                gyroSim.getMeasuredAngularVelocity().in(Units.RadiansPerSecond);
        inputs.odometryYawTimestamps = new double[] {RobotController.getFPGATime() / 1e6};
        inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
    }
}
