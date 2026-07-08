package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public class FeederInputs {
    public MutAngularVelocity feederVelocity;
    public MutAngularVelocity feederSetPoint;
    public MutVoltage feederVoltage;
    public MutCurrent feederSupplyCurrent;
    public MutCurrent feederTorqueCurrent;
  }

  public default void setFeederTarget(AngularVelocity velocity) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(FeederInputs inputs) {}
  ;

  public default void setFeederGains(Gains gains) {}
  ;
}
