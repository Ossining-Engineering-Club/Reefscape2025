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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AutoTeleopConstants.AlignmentConfig;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToProcessorPosition;
import frc.robot.commands.IntakeGroundAlgae;
import frc.robot.commands.IntakeReefAlgae;
import frc.robot.subsystems.coralpivot.CoralPivot;
import frc.robot.subsystems.coralpivot.CoralPivotIO;
import frc.robot.subsystems.coralpivot.CoralPivotIOReal;
import frc.robot.subsystems.coralpivot.CoralPivotIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeonIMU;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivot;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotIO;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotIOReal;
import frc.robot.subsystems.groundintakepivot.GroundIntakePivotIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.io.IOException;
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
  private final Vision vision;
  private final CoralPivot coralPivot;
  private final GroundIntakePivot groundIntakePivot;
  private final Elevator elevator;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController buttonBox = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() throws IOException, ParseException {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                VisionConstants.FRONT_LEFT_CAMERA_CONFIG,
                VisionConstants.FRONT_RIGHT_CAMERA_CONFIG,
                VisionConstants.BACK_LEFT_CAMERA_CONFIG,
                VisionConstants.BACK_RIGHT_CAMERA_CONFIG);
        drive =
            new Drive(
                new GyroIOPigeonIMU(),
                new ModuleIOReal(0),
                new ModuleIOReal(1),
                new ModuleIOReal(2),
                new ModuleIOReal(3),
                vision);
        coralPivot = new CoralPivot(new CoralPivotIOReal());
        groundIntakePivot = new GroundIntakePivot(new GroundIntakePivotIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                VisionConstants.FRONT_LEFT_CAMERA_CONFIG,
                VisionConstants.FRONT_RIGHT_CAMERA_CONFIG,
                VisionConstants.BACK_LEFT_CAMERA_CONFIG,
                VisionConstants.BACK_RIGHT_CAMERA_CONFIG);
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                vision);
        coralPivot = new CoralPivot(new CoralPivotIOSim());
        groundIntakePivot = new GroundIntakePivot(new GroundIntakePivotIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        vision =
            new Vision(
                VisionConstants.FRONT_LEFT_CAMERA_CONFIG,
                VisionConstants.FRONT_RIGHT_CAMERA_CONFIG,
                VisionConstants.BACK_LEFT_CAMERA_CONFIG,
                VisionConstants.BACK_RIGHT_CAMERA_CONFIG);
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision);
        coralPivot = new CoralPivot(new CoralPivotIO() {});
        groundIntakePivot = new GroundIntakePivot(new GroundIntakePivotIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
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
  private void configureButtonBindings() throws FileVersionException, IOException, ParseException {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -0.5 * controller.getLeftY(),
            () -> -0.5 * controller.getLeftX(),
            () -> -0.5 * controller.getRightX()));

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

    // Reset gyro to 0° when B button is pressed
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // controller.b().onTrue(Commands.runOnce(() -> {}, drive));

    controller.x().onTrue(new GoToProcessorPosition(coralPivot, groundIntakePivot, elevator));
    controller.y().onTrue(new IntakeGroundAlgae(groundIntakePivot, coralPivot, elevator));
    controller
        .b()
        .onTrue(
            new IntakeReefAlgae(
                ElevatorConstants.upperAlgaeHeight, coralPivot, groundIntakePivot, elevator));

    // Pathfinding
    for (AlignmentConfig alignmentConfig : AutoTeleopConstants.reefAlignmentConfigs) {
      PathPlannerPath path = PathPlannerPath.fromPathFile(alignmentConfig.pathName());
      Command pathFindingCommand =
          AutoBuilder.pathfindThenFollowPath(path, AutoTeleopConstants.alignmentConstraints);
      buttonBox.button(alignmentConfig.button()).onTrue(pathFindingCommand);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // defines the poses for each component of the robot model in Advantage Scope
  public void robotContainerPeriodic() {
    Logger.recordOutput(
        "Zeroed Component Poses",
        new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()});
    Logger.recordOutput(
        "Final Component Poses",
        new Pose3d[] {
          new Pose3d(0.1016, 0, 0.1439333418 + elevator.getHeight() / 2.0, new Rotation3d(0, 0, 0)),
          new Pose3d(0.1016, 0, 0.1959848252 + elevator.getHeight(), new Rotation3d(0, 0, 0)),
          new Pose3d(0.0873125, 0, 0.2404348252 + elevator.getHeight(), new Rotation3d(0, 0, 0)),
          new Pose3d(
              0.291373916,
              0,
              0.6305888582 + elevator.getHeight(),
              new Rotation3d(0, -(coralPivot.getAngle() - Math.PI / 2.0), 0)),
          new Pose3d(
              0.2873375,
              0,
              0.2328333418,
              new Rotation3d(
                  0,
                  -(groundIntakePivot.getAngle() - Math.PI / 2.0) + Units.degreesToRadians(-15),
                  0)),
        });
  }
}
