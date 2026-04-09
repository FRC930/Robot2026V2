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
          "Gains/Feeder/", 0.245, 0.0, 0.005, 0.275, 0.0, 0.13, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Jam detection tunable thresholds
  // private final LoggedTunableNumber m_jamVelocityThreshold =
  //     new LoggedTunableNumber("Indexer/JamVelocityThresholdRPM", 10.0);
  // private final LoggedTunableNumber m_jamCurrentThreshold =
  //     new LoggedTunableNumber("Indexer/JamCurrentThresholdAmps", 150.0);
  // private final LoggedTunableNumber m_jamDetectionTimeSec =
  //     new LoggedTunableNumber("Indexer/JamDetectionTimeSec", 0.2);
  // private final LoggedTunableNumber m_autoReverseTimeSec =
  //     new LoggedTunableNumber("Indexer/AutoReverseTimeSec", 0.4);
  // private final LoggedTunableNumber m_maxJamRetries =
  //     new LoggedTunableNumber("Indexer/JamMaxRetries", 3);

  // Jam detection internal state
  // private final Timer m_stallTimer = new Timer();
  // private final Timer m_autoReverseTimer = new Timer();
  // private boolean m_isStalling = false;
  // private boolean m_isAutoReversing = false;
  // private int m_jamRetryCount = 0;

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

  // TODO should we make a seperate stop for feeder
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
        // resetJamDetection();
        m_IO.stop();
        break;
      case FEEDING:
        // checkForJam();
        state = m_state.get();
        m_IO.setFeederTarget(state.feederVelocity());
        break;
      case INTAKING:
        m_IO.setFeederTarget(state.feederVelocity());
        break;
      default:
        break;
    }

    // Logger.recordOutput("Indexer/IsAutoReversing", m_isAutoReversing);
    // Logger.recordOutput("Indexer/IsStalling", m_isStalling);
    // Logger.recordOutput("Indexer/JamRetryCount", m_jamRetryCount);

    m_feederTunableGains.ifGainsHaveChanged((gains) -> m_IO.setFeederGains(gains));
  }

  // private void checkForJam() {
  //   double velocity = Math.abs(m_logged.indexerVelocity.in(RPM));
  //   double current = Math.abs(m_logged.indexerTorqueCurrent.in(Amps));

  //   boolean stalled =
  //       velocity < m_jamVelocityThreshold.get() || current > m_jamCurrentThreshold.get();

  //   if (stalled) {
  //     if (!m_isStalling) {
  //       m_stallTimer.restart();
  //       m_isStalling = true;
  //     } else if (m_stallTimer.hasElapsed(m_jamDetectionTimeSec.get())
  //         && m_jamRetryCount < (int) m_maxJamRetries.get()) {
  //       m_state.set(IndexerState.REVERSING);
  //       m_autoReverseTimer.restart();
  //       m_isAutoReversing = true;
  //       m_jamRetryCount++;
  //       m_isStalling = false;
  //     }
  //   } else {
  //     m_isStalling = false;
  //     m_jamRetryCount = 0;
  //   }
  // }

  // private void resetJamDetection() {
  //   m_isStalling = false;
  //   m_isAutoReversing = false;
  //   m_jamRetryCount = 0;
  //   m_stallTimer.stop();
  //   m_autoReverseTimer.stop();
  // }

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
