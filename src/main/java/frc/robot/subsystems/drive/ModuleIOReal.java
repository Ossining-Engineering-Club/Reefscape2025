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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOReal implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkMax turnSparkMax;
  private final RelativeEncoder turnEncoder;
  private final CANcoder absEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController turnController;

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
    turnSparkMax =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnEncoder = turnSparkMax.getEncoder();
    absEncoder =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftAbsCanId;
              case 1 -> frontRightAbsCanId;
              case 2 -> backLeftAbsCanId;
              case 3 -> backRightAbsCanId;
              default -> 0;
            });
    turnController = turnSparkMax.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveSensorMechanismRatio;
    driveTalon.getConfigurator().apply(driveConfig, 0.25);
    driveTalon.setPosition(0.0, 0.25);

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(1.0 / turnMotorReduction * turnEncoderPositionFactor)
        .velocityConversionFactor(1.0 / turnMotorReduction * turnEncoderVelocityFactor);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp, 0.0, turnKd, 0.0);
    turnConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnEncoder.setPosition(absEncoder.getPosition().getValueAsDouble());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.drivePositionRad = driveTalon.getPosition().getValueAsDouble();
    inputs.driveVelocityRadPerSec = driveTalon.getVelocity().getValueAsDouble();
    inputs.driveAppliedVolts = driveTalon.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveTalon.getSupplyCurrent().getValueAsDouble();

    // Update turn inputs
    inputs.turnPosition = new Rotation2d(turnEncoder.getPosition()).minus(zeroRotation);
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveTalon.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnSparkMax.setVoltage(voltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setReference(setpoint, ControlType.kPosition);
  }
}
