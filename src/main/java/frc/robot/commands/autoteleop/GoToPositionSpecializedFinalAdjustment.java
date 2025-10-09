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

public class GoToPositionSpecializedFinalAdjustment extends Command {
    private final Drive drive;
    private final Vision vision;
    private final Position position;
    private final double sidewaysOffset;
    private final double depthOffset;

    private final PIDController xpid;
    private final PIDController ypid;
    private final PIDController rotpid;

    private Optional<Pose2d> targetPose;

    // private final LinearFilter xfilter = LinearFilter.movingAverage(5);
    // private final LinearFilter yfilter = LinearFilter.movingAverage(5);

    public GoToPositionSpecializedFinalAdjustment(
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

        // double[] xForFilter = new double[5];
        // double[] yForFilter = new double[5];
        // for (int i = 0; i < 5; i++) xForFilter[i] = drive.getSpecializedPose().getX();
        // for (int i = 0; i < 5; i++) yForFilter[i] = drive.getSpecializedPose().getY();

        // xfilter.reset(xForFilter, new double[] {});
        // yfilter.reset(yForFilter, new double[] {});

        Logger.recordOutput("target pose", targetPose.get());

        vision.setFocusTag(getTagIdOfPosition(position));
    }

    @Override
    public void execute() {
        // double filteredX = xfilter.calculate(drive.getSpecializedPose().getX());
        // double filteredY = yfilter.calculate(drive.getSpecializedPose().getY());

        // Logger.recordOutput("filteredX", filteredX);
        // Logger.recordOutput("filteredY", filteredY);

        // drive.runVelocityFieldRelative(
        //         new ChassisSpeeds(
        //                 xpid.calculate(drive.getSpecializedPose().getX())
        //                         + xpid.getSetpoint().velocity,
        //                 ypid.calculate(drive.getSpecializedPose().getY())
        //                         + ypid.getSetpoint().velocity,
        //                 rotpid.calculate(drive.getRotation().getRadians())
        //                         + rotpid.getSetpoint().velocity));

        var xpidOutput = xpid.calculate(drive.getSpecializedPose().getX());
        var ypidOutput = ypid.calculate(drive.getSpecializedPose().getY());
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
                                    drive.getSpecializedPose().getX() - targetPose.get().getX(),
                                    drive.getSpecializedPose().getY() - targetPose.get().getY())
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
                                    drive.getSpecializedPose().getX() - targetPose.get().getX(),
                                    drive.getSpecializedPose().getY() - targetPose.get().getY())
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
                                    drive.getSpecializedPose().getX() - targetPose.get().getX(),
                                    drive.getSpecializedPose().getY() - targetPose.get().getY())
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
                                    drive.getSpecializedPose().getX() - targetPose.get().getX(),
                                    drive.getSpecializedPose().getY() - targetPose.get().getY())
                            <= AutoTeleopConstants.coralFinalTranslationalTolerance
                    && Math.abs(
                                    drive.getRotation().getRadians()
                                            - targetPose.get().getRotation().getRadians())
                            <= AutoTeleopConstants.coralFinalRotationalTolerance
                    && Math.hypot(
                                    drive.getChassisSpeeds().vxMetersPerSecond,
                                    drive.getChassisSpeeds().vyMetersPerSecond)
                            <= AutoTeleopConstants.coralFinalTranslationalVelocityTolerance
                    && drive.getChassisSpeeds().omegaRadiansPerSecond
                            <= AutoTeleopConstants.coralFinalRotationVelocityTolerance) {
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
