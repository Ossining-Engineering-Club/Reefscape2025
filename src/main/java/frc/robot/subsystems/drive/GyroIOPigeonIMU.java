// Copyright 2021-2024 FRC 6328
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
