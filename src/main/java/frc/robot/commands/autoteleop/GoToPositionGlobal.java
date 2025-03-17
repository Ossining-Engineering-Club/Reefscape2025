package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.getTagIdOfPosition;
import static frc.robot.subsystems.vision.VisionConstants.TAG_LAYOUT;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.Position;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class GoToPositionGlobal extends Command {
    private final Drive drive;
    private final Position position;
    private final double sidewaysOffset;
    private final double depthOffset;
    private final PathConstraints pathConstraints;

    private final ProfiledPIDController xpid;
    private final ProfiledPIDController ypid;
    private final ProfiledPIDController rotpid;

    private Optional<Pose2d> targetPose;

    public GoToPositionGlobal(
            Drive drive,
            Position position,
            double sidewaysOffset,
            double depthOffset,
            PathConstraints pathConstraints) {
        this.drive = drive;
        this.position = position;
        this.sidewaysOffset = sidewaysOffset;
        this.depthOffset = depthOffset;
        this.pathConstraints = pathConstraints;

        xpid =
                new ProfiledPIDController(
                        7.0,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                pathConstraints.maxVelocityMPS(),
                                pathConstraints.maxAccelerationMPSSq()));
        ypid =
                new ProfiledPIDController(
                        7.0,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                pathConstraints.maxVelocityMPS(),
                                pathConstraints.maxAccelerationMPSSq()));
        rotpid =
                new ProfiledPIDController(
                        2.0,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                pathConstraints.maxAngularVelocityRadPerSec(),
                                pathConstraints.maxAngularAccelerationRadPerSecSq()));
        rotpid.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (position == Position.PROCESSOR) {
            targetPose =
                    getTargetPose(getTagIdOfPosition(position), sidewaysOffset, depthOffset, true);
        } else {
            targetPose =
                    getTargetPose(getTagIdOfPosition(position), sidewaysOffset, depthOffset, false);
        }
        xpid.reset(drive.getPose().getX(), drive.getFieldRelativeChassisSpeeds().vxMetersPerSecond);
        ypid.reset(drive.getPose().getY(), drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond);
        rotpid.reset(
                drive.getPose().getRotation().getRadians(),
                drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);

        xpid.setGoal(targetPose.get().getX());
        ypid.setGoal(targetPose.get().getY());
        rotpid.setGoal(targetPose.get().getRotation().getRadians());

        Logger.recordOutput("target pose", targetPose.get());
    }

    @Override
    public void execute() {
        drive.runVelocityFieldRelative(
                new ChassisSpeeds(
                        xpid.calculate(drive.getPose().getX()) + xpid.getSetpoint().velocity,
                        ypid.calculate(drive.getPose().getY()) + ypid.getSetpoint().velocity,
                        rotpid.calculate(drive.getRotation().getRadians())
                                + rotpid.getSetpoint().velocity));

        Logger.recordOutput("xpid setpoint", xpid.getSetpoint().position);
        Logger.recordOutput("ypid setpoint", ypid.getSetpoint().position);
        Logger.recordOutput("rotpid setpoint", rotpid.getSetpoint().position);
        Logger.recordOutput("xpid setpoint velocity", xpid.getSetpoint().position);
        Logger.recordOutput("ypid setpoint velocity", ypid.getSetpoint().position);
        Logger.recordOutput("rotpid setpoint velocity", rotpid.getSetpoint().position);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        if (!targetPose.isPresent()) return true;
        if (position == Position.PROCESSOR) {
            if (Math.hypot(
                                    drive.getPose().getX() - targetPose.get().getX(),
                                    drive.getPose().getY() - targetPose.get().getY())
                            <= AutoTeleopConstants.processorTranslationalTolerance
                    && Math.abs(
                                    drive.getRotation().getRadians()
                                            - targetPose.get().getRotation().getRadians())
                            <= AutoTeleopConstants.processorRotationalTolerance) {
                return true;
            }
            return false;
        } else if (position == Position.LEFT_CORAL_STATION
                || position == Position.RIGHT_CORAL_STATION) {
            if (Math.hypot(
                                    drive.getPose().getX() - targetPose.get().getX(),
                                    drive.getPose().getY() - targetPose.get().getY())
                            <= AutoTeleopConstants.coralStationTranslationalTolerance
                    && Math.abs(
                                    drive.getRotation().getRadians()
                                            - targetPose.get().getRotation().getRadians())
                            <= AutoTeleopConstants.coralStationRotationalTolerance) {
                return true;
            }
            return false;
        } else {
            if (Math.hypot(
                                    drive.getPose().getX() - targetPose.get().getX(),
                                    drive.getPose().getY() - targetPose.get().getY())
                            <= AutoTeleopConstants.translationalTolerance
                    && Math.abs(
                                    drive.getRotation().getRadians()
                                            - targetPose.get().getRotation().getRadians())
                            <= AutoTeleopConstants.rotationalTolerance) {
                return true;
            }
            return false;
        }
    }

    public static Optional<Pose2d> getTargetPose(
            int tagId, double sidewaysOffset, double depthOffset, boolean flipRobot) {
        var optionalTagPose = TAG_LAYOUT.getTagPose(tagId);
        if (optionalTagPose.isPresent()) {
            var tagPose = optionalTagPose.get();
            return Optional.ofNullable(
                    new Pose2d(
                            tagPose.getX()
                                    + sidewaysOffset
                                            * Math.cos(tagPose.getRotation().getZ() - Math.PI / 2.0)
                                    + depthOffset * Math.cos(tagPose.getRotation().getZ()),
                            tagPose.getY()
                                    + sidewaysOffset
                                            * Math.sin(tagPose.getRotation().getZ() - Math.PI / 2.0)
                                    + depthOffset * Math.sin(tagPose.getRotation().getZ()),
                            new Rotation2d(
                                    MathUtil.angleModulus(
                                            tagPose.getRotation().getZ()
                                                    - Math.PI / 2.0
                                                    + (flipRobot ? Math.PI : 0)))));
        }
        return Optional.empty();
    }
}
