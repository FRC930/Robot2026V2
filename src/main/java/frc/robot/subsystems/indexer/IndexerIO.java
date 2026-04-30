package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    public MutAngularVelocity kickerVelocity;
    public MutAngularVelocity kickerSetPoint;
    public MutVoltage kickerVoltage;
    public MutCurrent kickerSupplyCurrent;
    public MutCurrent kickerTorqueCurrent;
  }

  public default void setKickerTarget(AngularVelocity velocity) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(IndexerInputs inputs) {}
  ;

  public default void setKickerGains(Gains gains) {}
  ;
}
