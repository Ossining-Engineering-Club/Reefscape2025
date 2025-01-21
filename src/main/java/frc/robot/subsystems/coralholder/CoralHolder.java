package frc.robot.subsystems.coralholder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import static frc.robot.subsystems.coralholder.CoralHolderConstants.k_beamId;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.k_powerPercent;

import org.littletonrobotics.junction.Logger;
import frc.robotics.subsytems.breakbeam;

public class CoralHolder extends SubsystemBase {
   private final CoralHolderIO io;

   private final breakbeam holderBeam;

   private final CoralHolderIOInputsAutoLogged inputs = new 

    public CoralHolder() {
        this.io = io;
        holderBeam = new BreakBeam(io, k_beamId);
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Coral Holder", inputs);

        if (holderBeam.isTripped()) turnOff();
    }

    public double turnOff() {
        io.setVoltage(0);
    }

    public double turnOn() {
        io.setVoltage(1*k_powerPercent);
    }

    public void breakBeamCheck() {
        return holderBeam.isTripped();
    }
}
