package frc.robot.subsystems.algaeclaw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeClaw extends SubsystemBase {

    private final SparkMax clawMotor;

    /** Algae Claw construction */
    public AlgaeClaw() {
      clawMotor = new SparkMax(AlgaeClawConstants.clawCANID, MotorType.kBrushless);
    }
  
    /** Sets motor voltage to predefined voltage */
    public void startMotorForwards() {
      clawMotor.setVoltage(AlgaeClawConstants.clawVoltage);
    }
  
    /** Sets motor voltage to zero */
    public void stopMotor() {
      clawMotor.setVoltage(0.0);
    }
}
