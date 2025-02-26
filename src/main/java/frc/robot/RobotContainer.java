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

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.AutoTeleopConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.AutoTeleopConstants.AlignmentConfig;
import frc.robot.AutoTeleopConstants.Level;
import frc.robot.AutoTeleopConstants.ReefAlgaeAlignmentConfig;
import frc.robot.Constants.Mode;
import frc.robot.commands.autoteleop.AutoGetCoral;
import frc.robot.commands.autoteleop.AutoGetReefAlgae;
import frc.robot.commands.autoteleop.AutoNetAlgae;
import frc.robot.commands.autoteleop.AutoPlaceCoral;
import frc.robot.commands.autoteleop.AutoProcessAlgae;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.elevator.ElevatorGoToHeight;
import frc.robot.subsystems.algaeclaw.AlgaeClaw;
import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.algaeclaw.AlgaeClawIO;
import frc.robot.subsystems.algaeclaw.AlgaeClawIOReal;
import frc.robot.subsystems.algaeclaw.AlgaeClawIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolderConstants;
import frc.robot.subsystems.coralholder.CoralHolderIO;
import frc.robot.subsystems.coralholder.CoralHolderIOReal;
import frc.robot.subsystems.coralholder.CoralHolderIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeonIMU;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.gamepiecevisualizers.AlgaeVisualizer;
import frc.robot.subsystems.gamepiecevisualizers.AlgaeVisualizer.AlgaeState;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer.CoralState;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIO;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIOReal;
import frc.robot.subsystems.photoelectricsensor.PhotoelectricSensorIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import java.io.IOException;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private SwerveDriveSimulation driveSimulation = null;
    private final Vision vision;
    private final Pivot pivot;
    private final Elevator elevator;
    private final CoralHolder coralHolder;
    private final AlgaeClaw algaeClaw;
    private final Climber climber;
    private final LED led;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController mechanismController = new CommandXboxController(1);
    private final CommandXboxController buttonBox = new CommandXboxController(2);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() throws IOException, ParseException {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                vision =
                        new Vision(
                                new VisionIOReal(VisionConstants.FRONT_LEFT_CAMERA_CONFIG),
                                new VisionIOReal(VisionConstants.FRONT_RIGHT_CAMERA_CONFIG),
                                new VisionIOReal(VisionConstants.BACK_LEFT_CAMERA_CONFIG),
                                new VisionIOReal(VisionConstants.BACK_RIGHT_CAMERA_CONFIG));
                drive =
                        new Drive(
                                new GyroIOPigeonIMU(),
                                new ModuleIOReal(0),
                                new ModuleIOReal(1),
                                new ModuleIOReal(2),
                                new ModuleIOReal(3),
                                vision,
                                (robotPose) -> {});
                pivot = new Pivot(new PivotIOTalonFX());
                elevator = new Elevator(new ElevatorIOReal());
                coralHolder =
                        new CoralHolder(
                                new CoralHolderIOReal(),
                                new PhotoelectricSensorIOReal(
                                        CoralHolderConstants.coralHolderPEChannel));
                algaeClaw =
                        new AlgaeClaw(
                                new AlgaeClawIOReal(),
                                new PhotoelectricSensorIOReal(
                                        AlgaeClawConstants.algaeClawPEChannel));
                climber = new Climber(new ClimberIOReal());

                led = new LED(new LEDIOReal(), algaeClaw, coralHolder);

                // pivot = new Pivot(new PivotIO() {});
                // elevator = new Elevator(new ElevatorIO() {});
                // coralHolder = new CoralHolder(new CoralHolderIO() {}, new PhotoelectricSensorIO()
                // {});
                // algaeClaw = new AlgaeClaw(new AlgaeClawIO() {}, new PhotoelectricSensorIO() {});
                // climber = new Climber(new ClimberIO() {});
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation =
                        new SwerveDriveSimulation(
                                Drive.mapleSimConfig, new Pose2d(0, 0, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                vision =
                        new Vision(
                                new VisionIOSim(
                                        VisionConstants.FRONT_LEFT_CAMERA_CONFIG,
                                        driveSimulation::getSimulatedDriveTrainPose),
                                new VisionIOSim(
                                        VisionConstants.FRONT_RIGHT_CAMERA_CONFIG,
                                        driveSimulation::getSimulatedDriveTrainPose),
                                new VisionIOSim(
                                        VisionConstants.BACK_LEFT_CAMERA_CONFIG,
                                        driveSimulation::getSimulatedDriveTrainPose),
                                new VisionIOSim(
                                        VisionConstants.BACK_RIGHT_CAMERA_CONFIG,
                                        driveSimulation::getSimulatedDriveTrainPose));
                drive =
                        new Drive(
                                new GyroIOSim(driveSimulation.getGyroSimulation()),
                                new ModuleIOSim(driveSimulation.getModules()[0]),
                                new ModuleIOSim(driveSimulation.getModules()[1]),
                                new ModuleIOSim(driveSimulation.getModules()[2]),
                                new ModuleIOSim(driveSimulation.getModules()[3]),
                                vision,
                                driveSimulation::setSimulationWorldPose);
                pivot = new Pivot(new PivotIOSim());
                elevator = new Elevator(new ElevatorIOSim());
                coralHolder =
                        new CoralHolder(
                                new CoralHolderIOSim(),
                                new PhotoelectricSensorIOSim(CoralHolderConstants.coralHolderPEId));
                algaeClaw =
                        new AlgaeClaw(
                                new AlgaeClawIOSim(),
                                new PhotoelectricSensorIOSim(AlgaeClawConstants.algaeClawPEID));
                climber = new Climber(new ClimberIO() {});
                led = new LED(new LEDIO() {}, algaeClaw, coralHolder);
                break;

            default:
                // Replayed robot, disable IO implementations
                vision =
                        new Vision(
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {});
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                vision,
                                (robotPose) -> {});
                pivot = new Pivot(new PivotIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                coralHolder =
                        new CoralHolder(new CoralHolderIO() {}, new PhotoelectricSensorIO() {});
                algaeClaw = new AlgaeClaw(new AlgaeClawIO() {}, new PhotoelectricSensorIO() {});
                climber = new Climber(new ClimberIO() {});
                led = new LED(new LEDIO() {}, algaeClaw, coralHolder);
                break;
        }

        // Configure the button bindings
        configureButtonBindings();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        // autoChooser.addOption(
        //     "Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption(
        //     "Drive Simple FF Characterization",
        // DriveCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Forward)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Reverse)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Forward)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Reverse)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     *
     * @throws ParseException
     * @throws IOException
     * @throws FileVersionException
     */
    private void configureButtonBindings()
            throws FileVersionException, IOException, ParseException {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -0.8 * controller.getLeftY(),
                        () -> -0.8 * controller.getLeftX(),
                        () -> -0.8 * controller.getRightX()));

        // Lock to 0° when A button is held
        // controller
        //     .a()
        //     .whileTrue(
        //         DriveCommands.joystickDriveAtAngle(
        //             drive,
        //             () -> -controller.getLeftY(),
        //             () -> -controller.getLeftX(),
        //             () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when A button is pressed
        controller
                .a()
                .onTrue(
                        Commands.runOnce(
                                        () ->
                                                drive.setPose(
                                                        new Pose2d(
                                                                drive.getPose().getTranslation(),
                                                                new Rotation2d())),
                                        drive)
                                .ignoringDisable(true));

        // controller
        //         .b()
        //         .onTrue(
        //                 Commands.runOnce(
        //                         () -> stopEverything(),
        //                         drive,
        //                         pivot,
        //                         elevator,
        //                         coralHolder,
        //                         algaeClaw));

        // controller.x().onTrue(new IntakeCoral(pivot, elevator, coralHolder));
        // controller
        //     .y()
        //     .onTrue(new GoToPlacingCoralPosition(ElevatorConstants.l4Height, pivot, elevator));
        // controller.b().onTrue(new ReleaseCoral(coralHolder));

        // controller
        //     .x()
        //     .onTrue(
        //         Commands.runOnce(
        //             () -> (new AutoNetAlgae(pivot, elevator, algaeClaw,
        // drive.getPose())).schedule()));

        controller
                .x()
                .onTrue(
                        new AutoGetCoral(
                                coralStationAlignmentConfigs[0], pivot, elevator, coralHolder));
        controller
                .y()
                .onTrue(
                        new AutoPlaceCoral(
                                new AlignmentConfig("L", 0),
                                Level.L2,
                                pivot,
                                elevator,
                                coralHolder));
        controller
                .b()
                .onTrue(
                        new AutoPlaceCoral(
                                new AlignmentConfig("K", 0),
                                Level.L2,
                                pivot,
                                elevator,
                                coralHolder));
        // controller.y().onTrue(coralHolder.release());
        // controller.y().onTrue(algaeClaw.release());

        // manual mechanism control
        mechanismController
                .leftBumper()
                .onTrue(
                        Commands.runOnce(() -> coralHolder.forward(), coralHolder)
                                .andThen(Commands.waitUntil(() -> coralHolder.hasCoral()))
                                .andThen(Commands.runOnce(() -> coralHolder.stop(), coralHolder)));
        mechanismController
                .leftBumper()
                .onFalse(Commands.runOnce(() -> coralHolder.stop(), coralHolder));
        mechanismController
                .rightBumper()
                .onTrue(Commands.runOnce(() -> coralHolder.reverse(), coralHolder));
        mechanismController
                .rightBumper()
                .onFalse(Commands.runOnce(() -> coralHolder.stop(), coralHolder));

        mechanismController
                .leftTrigger(0.9)
                .onTrue(
                        Commands.runOnce(() -> algaeClaw.startMotor(), algaeClaw)
                                .andThen(Commands.waitUntil(() -> algaeClaw.hasAlgae()))
                                .andThen(
                                        Commands.waitTime(
                                                Seconds.of(AlgaeClawConstants.intakeDelaySeconds)))
                                .andThen(Commands.runOnce(() -> algaeClaw.stopMotor(), algaeClaw)));
        mechanismController
                .leftTrigger(0.9)
                .onFalse(Commands.runOnce(() -> algaeClaw.stopMotor(), algaeClaw));
        mechanismController
                .rightTrigger(0.9)
                .onTrue(Commands.runOnce(() -> algaeClaw.reverseMotor(), algaeClaw));
        mechanismController
                .rightTrigger(0.9)
                .onFalse(Commands.runOnce(() -> algaeClaw.stopMotor(), algaeClaw));

        // mechanismController.a().onTrue(Commands.runOnce(() -> elevator.setVoltage(0.25)));
        // mechanismController.a().onFalse(Commands.runOnce(() -> elevator.setVoltage(0.0)));
        mechanismController.a().onTrue(new ElevatorGoToHeight(elevator, 0.6));
        mechanismController.b().onTrue(new ElevatorGoToHeight(elevator, 0.1));
        // mechanismController.a().onTrue(new PivotGoToAngle(pivot, -3.922));
        // mechanismController.b().onTrue(new PivotGoToAngle(pivot, Units.degreesToRadians(46.0)));

        elevator.setDefaultCommand(
                Commands.runOnce(
                        () ->
                                elevator.setVoltage(
                                        0.2
                                                * 12.0
                                                * MathUtil.applyDeadband(
                                                        -mechanismController.getRightY(), 0.2)),
                        elevator));

        pivot.setDefaultCommand(
                Commands.runOnce(
                        () ->
                                pivot.setVoltage(
                                        0.5
                                                * 12.0
                                                * MathUtil.applyDeadband(
                                                        -mechanismController.getLeftY(), 0.2)),
                        pivot));

        mechanismController.povCenter().onTrue(Commands.runOnce(() -> climber.stop(), climber));
        mechanismController.povUp().onTrue(Commands.runOnce(() -> climber.reverse(), climber));
        mechanismController.povDown().onTrue(Commands.runOnce(() -> climber.forward(), climber));

        // Pathfinding
        for (AlignmentConfig config : reefCoralAlignmentConfigs) {
            NamedCommands.registerCommand(
                    config.pathName() + "_L1",
                    new AutoPlaceCoral(config, Level.L1, pivot, elevator, coralHolder));
            NamedCommands.registerCommand(
                    config.pathName() + "_L2",
                    new AutoPlaceCoral(config, Level.L2, pivot, elevator, coralHolder));
            NamedCommands.registerCommand(
                    config.pathName() + "_L3",
                    new AutoPlaceCoral(config, Level.L3, pivot, elevator, coralHolder));
            NamedCommands.registerCommand(
                    config.pathName() + "_L4",
                    new AutoPlaceCoral(config, Level.L4, pivot, elevator, coralHolder));
            buttonBox
                    .button(config.button())
                    .and(buttonBox.button(l1Button))
                    .onTrue(new AutoPlaceCoral(config, Level.L1, pivot, elevator, coralHolder));
            buttonBox
                    .button(config.button())
                    .and(buttonBox.button(l2Button))
                    .onTrue(new AutoPlaceCoral(config, Level.L2, pivot, elevator, coralHolder));
            buttonBox
                    .button(config.button())
                    .and(buttonBox.button(l3Button))
                    .onTrue(new AutoPlaceCoral(config, Level.L3, pivot, elevator, coralHolder));
            buttonBox
                    .button(config.button())
                    .and(buttonBox.button(l4Button))
                    .onTrue(new AutoPlaceCoral(config, Level.L4, pivot, elevator, coralHolder));
        }
        for (ReefAlgaeAlignmentConfig config : reefAlgaeAlignmentConfigs) {
            NamedCommands.registerCommand(
                    config.pathName() + "_Algae",
                    new AutoGetReefAlgae(config, pivot, elevator, algaeClaw));
            buttonBox
                    .button(config.button1())
                    .and(buttonBox.button(config.button2()))
                    .onTrue(new AutoGetReefAlgae(config, pivot, elevator, algaeClaw));
        }
        for (AlignmentConfig config : coralStationAlignmentConfigs) {
            NamedCommands.registerCommand(
                    config.pathName(), new AutoGetCoral(config, pivot, elevator, coralHolder));
            buttonBox
                    .button(config.button())
                    .onTrue(new AutoGetCoral(config, pivot, elevator, coralHolder));
        }
        NamedCommands.registerCommand(
                "Process Algae",
                new AutoProcessAlgae(processorAlignmentConfig, pivot, elevator, algaeClaw));
        buttonBox
                .button(processorAlignmentConfig.button())
                .onTrue(new AutoProcessAlgae(processorAlignmentConfig, pivot, elevator, algaeClaw));
        NamedCommands.registerCommand(
                "Net Algae",
                Commands.runOnce(
                        () ->
                                (new AutoNetAlgae(pivot, elevator, algaeClaw, drive.getPose()))
                                        .schedule()));
        buttonBox
                .button(netButton)
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        (new AutoNetAlgae(
                                                        pivot,
                                                        elevator,
                                                        algaeClaw,
                                                        drive.getPose()))
                                                .schedule()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (Constants.currentMode == Mode.SIM) {
            resetSimState();
        }
        return autoChooser.get();
    }

    public void resetSimState() {
        pivot.resetSimState();
        elevator.resetSimState();
        CoralVisualizer.setCoralState(CoralState.LOADED);
        AlgaeVisualizer.setAlgaeState(AlgaeState.GONE);
    }

    // stops everything except keeps algae claw holding voltage if on
    public void stopEverything() {
        algaeClaw.stopMotor();
        climber.stop();
        coralHolder.stop();
        drive.stop();
        elevator.stop();
        pivot.stop();
    }

    // defines the poses for each component of the robot model in Advantage Scope
    public void robotContainerPeriodic() {
        Logger.recordOutput(
                "Zeroed Component Poses",
                new Pose3d[] {
                    new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()
                });
        Logger.recordOutput(
                "Final Component Poses",
                new Pose3d[] {
                    new Pose3d(
                            -0.13335,
                            0,
                            0.1390922542 + elevator.getHeight() / 2.0,
                            new Rotation3d(0, 0, 0)),
                    new Pose3d(
                            -0.1317625,
                            0,
                            0.1835422542 + elevator.getHeight(),
                            new Rotation3d(0, 0, 0)),
                    new Pose3d(
                            -0.093574997 + 0.041099997 - 0.0127,
                            0,
                            0.7931422542 + elevator.getHeight(),
                            new Rotation3d(
                                    Units.degreesToRadians(53.763) - pivot.getAngle(), 0, 0)),
                });
        switch (Constants.currentMode) {
            case REAL:
                CoralVisualizer.update(
                        drive.getPose(),
                        elevator.getHeight(),
                        pivot.getAngle(),
                        drive.getChassisSpeeds());
                AlgaeVisualizer.update(
                        drive.getPose(),
                        elevator.getHeight(),
                        pivot.getAngle(),
                        drive.getChassisSpeeds());
                break;
            case SIM:
                Logger.recordOutput(
                        "SimTrueRobotPose", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                        "FieldSimulation/Algae",
                        SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
                Logger.recordOutput(
                        "FieldSimulation/Coral",
                        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
                CoralVisualizer.update(
                        driveSimulation.getSimulatedDriveTrainPose(),
                        elevator.getHeight(),
                        pivot.getAngle(),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
                AlgaeVisualizer.update(
                        driveSimulation.getSimulatedDriveTrainPose(),
                        elevator.getHeight(),
                        pivot.getAngle(),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
                if (DriverStation.isEnabled()) {
                    FieldSimulationManager.periodic(
                            driveSimulation.getSimulatedDriveTrainPose(),
                            elevator,
                            pivot,
                            coralHolder);
                }
                break;
            case REPLAY:
                CoralVisualizer.update(
                        drive.getPose(),
                        elevator.getHeight(),
                        pivot.getAngle(),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
                break;
            default:
                break;
        }
    }
}
