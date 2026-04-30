package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase implements IndexerEvents {

  private final IndexerIO m_IO;

  private final EnumState<IndexerState> m_state =
      new EnumState<>("Indexer/State", IndexerState.IDLE);

  private final IndexerInputsAutoLogged m_logged = new IndexerInputsAutoLogged();

  private final LoggedTunableGainsBuilder m_indexerTunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/Indexer/", 15.0, 0.0, 0.0, 16.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  private final LoggedTunableGainsBuilder m_kickerTunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/Kicker/", 50.0, 0, 0.0, 5.7, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Jam detection tunable thresholds
  private final LoggedTunableNumber m_jamVelocityThreshold =
      new LoggedTunableNumber("Indexer/JamVelocityThresholdRPM", 10.0);
  private final LoggedTunableNumber m_jamCurrentThreshold =
      new LoggedTunableNumber("Indexer/JamCurrentThresholdAmps", 150.0);
  private final LoggedTunableNumber m_jamDetectionTimeSec =
      new LoggedTunableNumber("Indexer/JamDetectionTimeSec", 0.2);
  private final LoggedTunableNumber m_autoReverseTimeSec =
      new LoggedTunableNumber("Indexer/AutoReverseTimeSec", 0.4);
  private final LoggedTunableNumber m_maxJamRetries =
      new LoggedTunableNumber("Indexer/JamMaxRetries", 3);

  // Jam detection internal state
  private final Timer m_stallTimer = new Timer();
  private final Timer m_autoReverseTimer = new Timer();
  private boolean m_isStalling = false;
  private boolean m_isAutoReversing = false;
  private int m_jamRetryCount = 0;

  public IndexerSubsystem(IndexerIO IO) {
    m_IO = IO;
    m_logged.kickerVoltage = Volts.mutable(0);
    m_logged.kickerSupplyCurrent = Amps.mutable(0);
    m_logged.kickerTorqueCurrent = Amps.mutable(0);
    m_logged.kickerVelocity = RPM.mutable(0);
    m_logged.kickerSetPoint = RPM.mutable(0);
    m_IO.setKickerGains(m_kickerTunableGains.build());
  }

  public void setTestingState() {
    m_state.set(IndexerState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(m_logged);
    Logger.processInputs("RobotState/Indexer", m_logged);

    IndexerState state = m_state.get();
    switch (state) {
      case IDLE:
        m_IO.stop();
        break;
      case FEEDING:
        state = m_state.get();
        m_IO.setKickerTarget(state.kickerVelocity());
        break;
      case REVERSING:
        m_IO.setKickerTarget(state.kickerVelocity());
        break;
      case INTAKING:
        m_IO.setKickerTarget(state.kickerVelocity());
        break;
      default:
        break;
    }

    Logger.recordOutput("Indexer/IsAutoReversing", m_isAutoReversing);
    Logger.recordOutput("Indexer/IsStalling", m_isStalling);
    Logger.recordOutput("Indexer/JamRetryCount", m_jamRetryCount);

    m_kickerTunableGains.ifGainsHaveChanged((gains) -> m_IO.setKickerGains(gains));
  }

  @Override
  public Trigger isIdleTrigger() {
    return m_state.is(IndexerState.IDLE);
  }

  @Override
  public Trigger isIndexingTrigger() {
    return m_state.is(IndexerState.FEEDING);
  }

  @Override
  public Trigger isIntakingTrigger() {
    return m_state.is(IndexerState.INTAKING);
  }

  public Command idleCommand() {
    return runOnce(() -> m_state.set(IndexerState.IDLE));
  }

  public Command indexingCommand() {
    return runOnce(
        () -> {
          m_state.set(IndexerState.FEEDING);
        });
  }

  public Command reverseCommand() {
    return runOnce(() -> m_state.set(IndexerState.REVERSING));
  }

  public Command intakingCommand() {
    return runOnce(() -> m_state.set(IndexerState.INTAKING));
  }

  public Command getNewSetKickerVelocityCommand(DoubleSupplier velocity) {
    return new InstantCommand(
        () -> {
          m_IO.setKickerTarget(RPM.of(velocity.getAsDouble()));
        },
        this);
  }
}
