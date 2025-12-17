package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class GoToCoral extends Command {
    private static final double TRANSLATIONAL_TOLERANCE = 0.03;

    private final Drive drive;
    private final Supplier<Translation2d> targetSupplier;
    private final PIDController translationPID = new PIDController(0, 0, 0);
    private final PIDController rotationPID = new PIDController(0, 0, 0);

    public GoToCoral(Drive drive, Supplier<Translation2d> targetSupplier) {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        
        translationPID.setSetpoint(0);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double translationalSpeed = Math.abs(translationPID.calculate(drive.getPose().getTranslation().minus(targetSupplier.get()).getNorm()));
        Rotation2d desiredAngle =
                    targetSupplier
                            .get()
                            .minus(drive.getPose().getTranslation())
                            .getAngle();
        Translation2d translationalVelocity = 
            new Translation2d(
                translationalSpeed * Math.cos(desiredAngle.getRadians()),
                translationalSpeed * Math.sin(desiredAngle.getRadians())
            );
        double rotationalVelocity = rotationPID.calculate(drive.getPose().getRotation().getRadians(), desiredAngle.getRadians());
        drive.runVelocityFieldRelative(new ChassisSpeeds(translationalVelocity.getX(), translationalVelocity.getY(), rotationalVelocity));
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return drive.getPose().getTranslation().getDistance(targetSupplier.get()) <= TRANSLATIONAL_TOLERANCE;
    }
}
