package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public MutAngularVelocity rollerVelocity;
    public MutVoltage rollerVelocitySetPoint;
    public MutCurrent rollerSupplyCurrent;
    public MutCurrent rollerTorqueCurrent;
    public MutVoltage rollerVoltage;
    public MutTemperature leaderRollerTemp;
    public MutTemperature followerRollerTemp;

    public int numberFuelHave;
  }

  public default void setRollerTargetSpeed(Voltage target) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(IntakeInputs input) {}
  ;

  public default void setRollerGains(Gains gains) {}
  ;
}
