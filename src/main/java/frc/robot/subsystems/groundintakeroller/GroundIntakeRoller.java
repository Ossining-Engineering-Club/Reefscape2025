package frc.robot.subsystems.groundintakeroller;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeRoller extends SubsystemBase {

  private final SparkMax rollerMotor;

  /** Ground intake roller construction */
  public GroundIntakeRoller() {
    rollerMotor = new SparkMax(GroundIntakeRollerConstants.RollerCANID, MotorType.kBrushless);
  }

  /** Sets motor voltage to predefined voltage */
  public void startMotor() {
    rollerMotor.setVoltage(GroundIntakeRollerConstants.motorVoltage);
  }

  /** Reverses intake motor */
  public void reverseMotor() {
    rollerMotor.setVoltage(GroundIntakeRollerConstants.reverseVoltage);
  }

  /** Sets motor voltage to zero */
  public void stopMotor() {
    rollerMotor.setVoltage(0.0);
  }
}
