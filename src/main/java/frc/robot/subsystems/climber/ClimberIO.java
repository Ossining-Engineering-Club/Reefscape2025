package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double chainMotorAppliedVolts = 0;
    public double ropeMotorAppliedVolts = 0;
    public double angleRadians = startAngle;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets voltage of chain motor */
  public default void setChainMotorVoltage(double voltage) {}

  /** Sets voltage of rope motor */
  public default void setRopeMotorVoltage(double voltage) {}
}
