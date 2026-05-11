package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public enum IndexerState {
  TESTING(() -> 0.0, () -> 0.0),
  IDLE(() -> 0.0, () -> 0.0),
  FEEDING(
      new LoggedTunableNumber("Indexer/setIndexerPointShooting", 10.0),
      new LoggedTunableNumber("Indexer/setKickerPointShooting", 4000.0)),
  REVERSING(
      new LoggedTunableNumber("Indexer/setIndexerPointReverse", -50),
      new LoggedTunableNumber("Indexer/setKickerPointReverse", -50)),
  INTAKING(
      new LoggedTunableNumber("Indexer/setIndexerPointIntaking", 0),
      new LoggedTunableNumber("Indexer/setKickerPointIntaking", 500.0));

  private DoubleSupplier m_indexerVelocity;
  private DoubleSupplier m_kickerVelocity;

  private IndexerState(DoubleSupplier indexerVelocity, DoubleSupplier kickerVelocity) {
    m_indexerVelocity = indexerVelocity;
    m_kickerVelocity = kickerVelocity;
  }

  public AngularVelocity indexerVelocity() {
    return RPM.of(m_indexerVelocity.getAsDouble());
  }

  public AngularVelocity kickerVelocity() {
    return RPM.of(m_kickerVelocity.getAsDouble());
  }
}
