package frc.robot.subsystems.extender;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {
  @AutoLog
  public static class ExtenderInputs {
    public MutDistance distance;
    public MutDistance followerDistance;
    public MutDistance differentialPositionError;
    public MutLinearVelocity velocity;
    public MutDistance setPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public default void setExtenderHeight(Distance target) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(ExtenderInputs input) {}
  ;

  public default void setGains(Gains gains) {}
  ;

  public default void setDifferentialGains(Gains gains) {}
  ;
}
