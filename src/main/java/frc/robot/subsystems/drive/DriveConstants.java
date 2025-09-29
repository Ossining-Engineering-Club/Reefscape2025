package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public final class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0;
    public static final double trackWidth = 0.455; // meters
    public static final double wheelBase = 0.55; // meters
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final double wheelRadiusMeters = 0.0967 / 2.0;
    public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            };

    public static final Current slipCurrent = Amps.of(120.0);

    // Drive motor configuration
    public static final int driveMotorStatorCurrentLimit = 80;
    public static final double driveMotorReduction = 6.75 / 1.0;
    public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

    // Drive encoder configuration
    public static final double driveSensorMechanismRatio =
            1 / (2 * Math.PI / driveMotorReduction); // Rotor Rotations -> Wheel Radians

    // Drive PID configuration
    public static final double driveP = 0.1;
    public static final double driveI = 0;
    public static final double driveD = 0;
    public static final double driveS = 0;
    public static final double driveV = 0.5;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final int turnMotorStatorCurrentLimit = 40;
    public static final double turnMotorReduction = 150.0 / 7.0;
    public static final DCMotor turnGearbox = DCMotor.getKrakenX60(1);

    // Turn encoder configuration
    public static final double turnSensorMechanismRatio =
            1 / (2 * Math.PI / turnMotorReduction); // Rotor Rotations -> Radians

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double coupleRatio = 0.14815;

    // Turn PID configuration
    public static final double turnP = 100;
    public static final double turnI = 0;
    public static final double turnD = 1;
    public static final double turnS = 0;
    public static final double turnV = 0;
    public static final double turnA = 0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = -Math.PI; // Radians
    public static final double turnPIDMaxInput = Math.PI; // Radians

    // These are only used for simulation
    public static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.004);
    public static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.025);
    // Simulated voltage necessary to overcome friction
    public static final Voltage steerFrictionVoltage = Volts.of(0.2);
    public static final Voltage driveFrictionVoltage = Volts.of(0.2);

    public static final int pigeonId = 13;

    public static final boolean invertLeftSide = false;
    public static final boolean invertRightSide = false;

    // Front Left
    public static final int frontLeftDriveMotorId = 2;
    public static final int frontLeftSteerMotorId = 6;
    public static final int frontLeftEncoderId = 10;
    public static final Angle frontLeftEncoderOffset = Radians.of(2.415);
    public static final boolean frontLeftSteerMotorInverted = true;
    public static final boolean frontLeftEncoderInverted = false;

    public static final Distance frontLeftXPos = Meters.of(moduleTranslations[0].getX());
    public static final Distance frontLeftYPos = Meters.of(moduleTranslations[0].getY());

    // Front Right
    public static final int frontRightDriveMotorId = 3;
    public static final int frontRightSteerMotorId = 7;
    public static final int frontRightEncoderId = 11;
    public static final Angle frontRightEncoderOffset = Radians.of(2.896 - Math.PI);
    public static final boolean frontRightSteerMotorInverted = true;
    public static final boolean frontRightEncoderInverted = false;

    public static final Distance frontRightXPos = Meters.of(moduleTranslations[1].getX());
    public static final Distance frontRightYPos = Meters.of(moduleTranslations[1].getY());

    // Back Left
    public static final int backLeftDriveMotorId = 1;
    public static final int backLeftSteerMotorId = 5;
    public static final int backLeftEncoderId = 9;
    public static final Angle backLeftEncoderOffset = Radians.of(2.950);
    public static final boolean backLeftSteerMotorInverted = true;
    public static final boolean backLeftEncoderInverted = false;

    public static final Distance backLeftXPos = Meters.of(moduleTranslations[2].getX());
    public static final Distance backLeftYPos = Meters.of(moduleTranslations[2].getY());

    // Back Right
    public static final int backRightDriveMotorId = 4;
    public static final int backRightSteerMotorId = 8;
    public static final int backRightEncoderId = 12;
    public static final Angle backRightEncoderOffset = Radians.of(0.510 - Math.PI);
    public static final boolean backRightSteerMotorInverted = true;
    public static final boolean backRightEncoderInverted = false;

    public static final Distance backRightXPos = Meters.of(moduleTranslations[3].getX());
    public static final Distance backRightYPos = Meters.of(moduleTranslations[3].getY());

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
                            DCMotor.getKrakenX60(1)
                                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                            slipCurrent.in(Amps),
                            1),
                    moduleTranslations);
}
