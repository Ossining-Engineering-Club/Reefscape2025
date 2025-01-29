package frc.robot.subsystems.algaeclaw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.breakbeam.Breakbeam;

public class AlgaeClaw extends SubsystemBase {

  private final SparkMax clawMotor;
  private final Breakbeam breakbeam;

  /** Algae Claw construction */
  public AlgaeClaw(Breakbeam breakbeam) {
    clawMotor = new SparkMax(AlgaeClawConstants.clawCANID, MotorType.kBrushless);
    this.breakbeam = breakbeam;
  }

  /** Sets motor voltage to predefined voltage */
  public void startMotor() {
    clawMotor.setVoltage(AlgaeClawConstants.clawVoltage);
  }

  /** Reverses algae claw motor */
  public void reverseMotor() {
    clawMotor.setVoltage(AlgaeClawConstants.reverseVoltage);
  }

  /** Sets motor voltage to zero */
  public void stopMotor() {
    clawMotor.setVoltage(0.0);
  }

  /** Gets state of AlgaeClaw breakbeam */
  public boolean isBBTripped() {
    return breakbeam.isTripped();
  }
}
