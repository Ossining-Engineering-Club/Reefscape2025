package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import edu.wpi.first.math.geometry.Rotation2d;

/** IO implementation for PigeonIMU */
public class GyroIOPigeonIMU implements GyroIO {
    private final PigeonIMU pigeon = new PigeonIMU(DriveConstants.pigeonCanId);

    public GyroIOPigeonIMU() {
        pigeon.setYaw(0.0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getState() == PigeonState.Ready;
        inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw());
    }
}
