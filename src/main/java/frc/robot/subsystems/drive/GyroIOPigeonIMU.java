// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIOPigeonIMU implements GyroIO {
    private final PigeonIMU pigeon = new PigeonIMU(TunerConstants.kPigeonId);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeonIMU() {
        pigeon.setYaw(0);

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon::getYaw);

        pigeon.setStatusFramePeriod(
                PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR,
                1000 / ((int) DriveConstants.odometryFrequency));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getState() == PigeonState.Ready;
        inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw());

        double[] xyz_dps = new double[3];
        pigeon.getRawGyro(xyz_dps);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyz_dps[2]);

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream()
                        .map((Double value) -> Rotation2d.fromDegrees(-value))
                        .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
