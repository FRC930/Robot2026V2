package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public enum FeederState {
  TESTING(() -> 0.0),
  IDLE(() -> 0.0),
  FEEDING(new LoggedTunableNumber("Feeder/setVelocity", 2500.0)),
  REVERSING(new LoggedTunableNumber("Feeder/setVelocityReverse", -1500.0)),
  INTAKING(new LoggedTunableNumber("Feeder/setVelocityIntaking", 0.0));

  private DoubleSupplier m_feederVelocity;

  private FeederState(DoubleSupplier feederVelocity) {
    m_feederVelocity = feederVelocity;
  }

  public AngularVelocity feederVelocity() {
    return RPM.of(m_feederVelocity.getAsDouble());
  }
}
