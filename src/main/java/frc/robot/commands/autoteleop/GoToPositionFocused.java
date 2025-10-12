package frc.robot.commands.autoteleop;

import static frc.robot.AutoTeleopConstants.getTagIdOfPosition;
import static frc.robot.subsystems.vision.VisionConstants.TAG_LAYOUT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTeleopConstants;
import frc.robot.AutoTeleopConstants.Position;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class GoToPositionFocused extends Command {
    private final Drive drive;
    private final Vision vision;
    private final Position position;
    private final double sidewaysOffset;
    private final double depthOffset;

    private final PIDController xpid;
    private final PIDController ypid;
    private final PIDController rotpid;

    private Optional<Pose2d> targetPose;

    public GoToPositionFocused(
            Drive drive,
            Vision vision,
            Position position,
            double sidewaysOffset,
            double depthOffset) {
        this.drive = drive;
        this.vision = vision;
        this.position = position;
        this.sidewaysOffset = sidewaysOffset;
        this.depthOffset = depthOffset;

        xpid = new PIDController(3.0, 0, 0);
        ypid = new PIDController(3.0, 0, 0);
        rotpid = new PIDController(2.0, 0, 0);
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

        xpid.setSetpoint(targetPose.get().getX());
        ypid.setSetpoint(targetPose.get().getY());
        rotpid.setSetpoint(targetPose.get().getRotation().getRadians());

        Logger.recordOutput("target pose", targetPose.get());

        drive.setFocusTag(getTagIdOfPosition(position));
    }

    @Override
    public void execute() {

        var xpidOutput = xpid.calculate(drive.getFocusedPose().getX());
        var ypidOutput = ypid.calculate(drive.getFocusedPose().getY());
        var rotpidOutput = rotpid.calculate(drive.getRotation().getRadians());

        drive.runVelocityFieldRelative(new ChassisSpeeds(xpidOutput, ypidOutput, rotpidOutput));

        Logger.recordOutput("xpid setpoint", xpid.getSetpoint());
        Logger.recordOutput("ypid setpoint", ypid.getSetpoint());
        Logger.recordOutput("rotpid setpoint", rotpid.getSetpoint());
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
                                    drive.getFocusedPose().getX() - targetPose.get().getX(),
                                    drive.getFocusedPose().getY() - targetPose.get().getY())
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
                                    drive.getFocusedPose().getX() - targetPose.get().getX(),
                                    drive.getFocusedPose().getY() - targetPose.get().getY())
                            <= AutoTeleopConstants.coralStationTranslationalTolerance
                    && Math.abs(
                                    drive.getRotation().getRadians()
                                            - targetPose.get().getRotation().getRadians())
                            <= AutoTeleopConstants.coralStationRotationalTolerance) {
                return true;
            }
            return false;
        } else if (position == Position.AB
                || position == Position.CD
                || position == Position.EF
                || position == Position.GH
                || position == Position.IJ
                || position == Position.KL) {
            if (Math.hypot(
                                    drive.getFocusedPose().getX() - targetPose.get().getX(),
                                    drive.getFocusedPose().getY() - targetPose.get().getY())
                            <= AutoTeleopConstants.reefAlgaeTranslationalTolerance
                    && Math.abs(
                                    drive.getRotation().getRadians()
                                            - targetPose.get().getRotation().getRadians())
                            <= AutoTeleopConstants.reefAlgaeRotationalTolerance) {
                return true;
            }
            return false;
        } else {
            if (Math.hypot(
                                    drive.getFocusedPose().getX() - targetPose.get().getX(),
                                    drive.getFocusedPose().getY() - targetPose.get().getY())
                            <= AutoTeleopConstants.coralTranslationalTolerance
                    && Math.abs(
                                    drive.getRotation().getRadians()
                                            - targetPose.get().getRotation().getRadians())
                            <= AutoTeleopConstants.coralRotationalTolerance
                    && Math.hypot(
                                    drive.getChassisSpeeds().vxMetersPerSecond,
                                    drive.getChassisSpeeds().vyMetersPerSecond)
                            <= AutoTeleopConstants.coralTranslationalVelocityTolerance
                    && drive.getChassisSpeeds().omegaRadiansPerSecond
                            <= AutoTeleopConstants.coralRotationVelocityTolerance) {
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
