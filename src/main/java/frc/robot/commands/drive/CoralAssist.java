package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class CoralAssist extends Command {
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final ObjectDetector objectDetector;

    PIDController angleController = new PIDController(2.0, 0.0, 0.0);

    public CoralAssist(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            ObjectDetector objectDetector) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.objectDetector = objectDetector;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Optional<Pose3d> targetCoral = objectDetector.getClosestCoralToIntake();
        if (targetCoral.isPresent()) {
            Rotation2d angle =
                    targetCoral
                            .get()
                            .getTranslation()
                            .toTranslation2d()
                            .minus(drive.getPose().getTranslation())
                            .getAngle();
            // Get linear velocity
            Translation2d linearVelocity =
                    DriveCommands.getLinearVelocityFromJoysticks(
                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

            // Calculate modified linear velocity so
            // that robot drives at a specific heading
            // and drives at a speed proportional to
            // the normal of the raw linear velocity
            double linearVelocityNorm = linearVelocity.getNorm();
            Rotation2d rawHeading = linearVelocity.getAngle();
            double scaledLinearVelocityNorm =
                    linearVelocityNorm * Math.cos(Math.abs(angle.minus(rawHeading).getRadians()));
            Translation2d modifiedLinearVelocity =
                    new Translation2d(
                            scaledLinearVelocityNorm * Math.cos(angle.getRadians()),
                            scaledLinearVelocityNorm * Math.sin(angle.getRadians()));

            // Calculate angular speed
            double omega =
                    angleController.calculate(drive.getRotation().getRadians(), angle.getRadians());

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds =
                    new ChassisSpeeds(
                            modifiedLinearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            modifiedLinearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);
            boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                    : drive.getRotation()));
        }
    }
}
