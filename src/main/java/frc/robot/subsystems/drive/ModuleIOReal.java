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

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOReal implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder absEncoder;

  // Voltage control requests
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  public ModuleIOReal(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> new Rotation2d();
        };
    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });
    turnTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            });
    absEncoder =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftAbsCanId;
              case 1 -> frontRightAbsCanId;
              case 2 -> backLeftAbsCanId;
              case 3 -> backRightAbsCanId;
              default -> 0;
            });

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = driveSensorMechanismRatio;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig, 0.25);
    driveTalon.setPosition(0.0, 0.25);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = new Slot0Configs().withKP(turnKp).withKD(turnKd);
    turnConfig.Feedback.SensorToMechanismRatio = turnSensorMechanismRatio;
    turnConfig.CurrentLimits.StatorCurrentLimit = turnMotorCurrentLimit;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        turnInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turnTalon.getConfigurator().apply(turnConfig, 0.25);
    turnTalon.setPosition(absEncoder.getPosition().getValueAsDouble() * 2 * Math.PI, 0.25);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.drivePositionRad = driveTalon.getPosition().getValueAsDouble();
    inputs.driveVelocityRadPerSec = driveTalon.getVelocity().getValueAsDouble();
    inputs.driveAppliedVolts = driveTalon.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveTalon.getSupplyCurrent().getValueAsDouble();

    // Update turn inputs
    inputs.turnPosition =
        new Rotation2d(turnTalon.getPosition().getValueAsDouble()).minus(zeroRotation);
    inputs.turnVelocityRadPerSec = turnTalon.getVelocity().getValueAsDouble();
    inputs.turnAppliedVolts = turnTalon.getMotorVoltage().getValueAsDouble();
    inputs.turnCurrentAmps = turnTalon.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveTalon.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnTalon.setVoltage(voltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
  }
}
