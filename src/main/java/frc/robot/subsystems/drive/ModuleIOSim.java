package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Physics sim implementation of module IO. */
public class ModuleIOSim extends ModuleIOTalonFX {
    private final SwerveModuleSimulation simulation;

    public ModuleIOSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
        super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

        this.simulation = simulation;
        simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

        simulation.useSteerMotorController(
                new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
    }
}
