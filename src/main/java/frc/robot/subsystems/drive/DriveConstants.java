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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 50.0; // Hz
    public static final double trackWidth = 0.455; // meters
    public static final double wheelBase = 0.55; // meters
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-2.415);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(-2.896 + Math.PI);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(-2.950);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(-0.510 + Math.PI);

    // Device CAN IDs
    public static final int pigeonCanId = 13;

    public static final int frontLeftDriveCanId = 2;
    public static final int frontRightDriveCanId = 3;
    public static final int backLeftDriveCanId = 1;
    public static final int backRightDriveCanId = 4;

    public static final int frontLeftTurnCanId = 6;
    public static final int frontRightTurnCanId = 7;
    public static final int backLeftTurnCanId = 5;
    public static final int backRightTurnCanId = 8;

    public static final int frontLeftAbsCanId = 10;
    public static final int frontRightAbsCanId = 11;
    public static final int backLeftAbsCanId = 9;
    public static final int backRightAbsCanId = 12;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = 0.0967 / 2.0;
    public static final double driveMotorReduction = (6.75 / 1.0);
    public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

    // Drive encoder configuration
    public static final double driveSensorMechanismRatio =
            1 / (2 * Math.PI / driveMotorReduction); // Rotor Rotations -> Wheel Radians

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.0;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 150.0 / 7.0;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    // Turn encoder configuration
    public static final double turnSensorMechanismRatio =
            1 / (2 * Math.PI / turnMotorReduction); // Rotor Rotations -> Radians

    // Turn PID configuration
    public static final double turnKp = 4.0; // 8.0; // 0.45;
    public static final double turnKd = 0.003; // 0.0; // 0.00025;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = -Math.PI; // Radians
    public static final double turnPIDMaxInput = Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 52.806;
    public static final double robotMOI = 4.14;
    public static final double wheelCOF = 1.0;
    public static final RobotConfig ppConfig =
            new RobotConfig(
                    robotMassKg,
                    robotMOI,
                    new ModuleConfig(
                            wheelRadiusMeters,
                            maxSpeedMetersPerSec,
                            wheelCOF,
                            driveGearbox.withReduction(driveMotorReduction),
                            driveMotorCurrentLimit,
                            1),
                    moduleTranslations);
}
