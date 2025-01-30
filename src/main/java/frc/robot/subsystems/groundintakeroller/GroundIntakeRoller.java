package frc.robot.subsystems.groundintakeroller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GroundIntakeRoller extends SubsystemBase {
  private final GroundIntakeRollerIO io;
  private final GroundIntakeRollerIOInputsAutoLogged inputs =
      new GroundIntakeRollerIOInputsAutoLogged();

  /** Ground intake roller construction */
  public GroundIntakeRoller(GroundIntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ground Intake Roller", inputs);
  }

  /** Sets motor voltage to predefined voltage */
  public void startMotor() {
    io.setVoltage(GroundIntakeRollerConstants.motorVoltage);
  }

  /** Reverses intake motor */
  public void reverseMotor() {
    io.setVoltage(GroundIntakeRollerConstants.reverseVoltage);
  }

  /** Sets motor voltage to zero */
  public void stopMotor() {
    io.setVoltage(0.0);
  }
}
