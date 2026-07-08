package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase implements FeederEvents {

  private final FeederIO m_IO;

  private final EnumState<FeederState> m_state = new EnumState<>("Feeder/State", FeederState.IDLE);

  private final FeederInputsAutoLogged m_logged = new FeederInputsAutoLogged();

  private final LoggedTunableGainsBuilder m_feederTunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/Feeder/", 1000.0, 0.0, 0.000, 15.0, 0.0, 0.13, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public FeederSubsystem(FeederIO IO) {
    m_IO = IO;

    m_logged.feederVoltage = Volts.mutable(0);
    m_logged.feederSupplyCurrent = Amps.mutable(0);
    m_logged.feederTorqueCurrent = Amps.mutable(0);
    m_logged.feederVelocity = RPM.mutable(0);
    m_logged.feederSetPoint = RPM.mutable(0);
    m_IO.setFeederGains(m_feederTunableGains.build());
  }

  public void setTestingState() {
    m_state.set(FeederState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(m_logged);
    Logger.processInputs("RobotState/Feeder", m_logged);

    FeederState state = m_state.get();
    switch (state) {
      case IDLE:
        m_IO.stop();
        break;
      case FEEDING:
        state = m_state.get();
        m_IO.setFeederTarget(state.feederVelocity());
        break;
      case INTAKING:
        m_IO.setFeederTarget(state.feederVelocity());
        break;
      default:
        break;
    }

    m_feederTunableGains.ifGainsHaveChanged((gains) -> m_IO.setFeederGains(gains));
  }

  @Override
  public Trigger isIdleTrigger() {
    return m_state.is(FeederState.IDLE);
  }

  @Override
  public Trigger isFeedingTrigger() {
    return m_state.is(FeederState.FEEDING);
  }

  public Command idleCommand() {
    return runOnce(() -> m_state.set(FeederState.IDLE));
  }

  public Command feedingCommand() {
    return runOnce(
        () -> {
          m_state.set(FeederState.FEEDING);
        });
  }

  public Command intakingCommand() {
    return runOnce(
        () -> {
          m_state.set(FeederState.INTAKING);
        });
  }

  public Command getNewSetFeederVelocityCommand(DoubleSupplier velocity) {
    return new InstantCommand(
        () -> {
          m_IO.setFeederTarget(RPM.of(velocity.getAsDouble()));
        },
        this);
  }
}
