package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase implements ShooterEvents {
  private final ShooterIO m_IO;
  private final LoggedTunableNumber setpoint = new LoggedTunableNumber("Shooter/setpoint", 2500);
  private final LoggedTunableNumber spunUpThreshold =
      new LoggedTunableNumber("Shooter/spunUpPercent", 0.80);
  private final LoggedTunableNumber spunUpDebounce =
      new LoggedTunableNumber("Shooter/spunUpDebounce", 0.08);
  private volatile boolean shouldThreadCommand = false;

  private final EnumState<ShooterState> m_state =
      new EnumState<>("Shooter/States", ShooterState.IDLE);

  private ShooterInputsAutoLogged logged = new ShooterInputsAutoLogged();

  private final DoubleSupplier shooterRPMSupplier;

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/ShooterSubsystem/", 0.9, 0, 0.04, 0.23, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public ShooterSubsystem(ShooterIO IO, DoubleSupplier shooterRPMSupplier) {
    m_IO = IO;
    this.shooterRPMSupplier = shooterRPMSupplier;
    logged.shooterAngularVelocity = RPM.mutable(0);
    logged.shooterSetpoint = RPM.mutable(2700.0);
    logged.shooterSupplyCurrent = Amps.mutable(0);
    logged.shooterTorqueCurrent = Amps.mutable(0);
    logged.shooterVoltage = Volts.mutable(0);
    this.m_IO.setGains(tunableGains.build());
  }

  /**
   * Sets the speed for the shooter.
   *
   * @param speed
   */
  public void setShooterSpeed(AngularVelocity speed) {
    m_IO.setShooterTarget(speed);
  }

  public Command shooterCommand() {
    return runOnce(
        () -> {
          m_state.set(ShooterState.SHOOTING);
        });
  }

  public Command prespinCommand() {
    return runOnce(
        () -> {
          m_state.set(ShooterState.PRESPIN);
        });
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          m_state.set(ShooterState.IDLE);
        });
  }

  public void setTestingState() {
    m_state.set(ShooterState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Shooter", logged);
    ShooterState state = m_state.get();
    shouldThreadCommand = (state == ShooterState.SHOOTING || state == ShooterState.PRESPIN);
    switch (state) {
      case SHOOTING:
      case PRESPIN:
        break; // 250Hz thread handles motor commands
      case IDLE:
        stop();
        break;
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
  }

  public boolean shouldThreadCommand() {
    return shouldThreadCommand;
  }

  public ShooterIO getIO() {
    return m_IO;
  }

  public double getPrespinSetpoint() {
    return setpoint.get();
  }

  @Override
  public Trigger isIdleTrigger() {
    return m_state.is(ShooterState.IDLE);
  }

  @Override
  public Trigger isShootingTrigger() {
    return m_state.is(ShooterState.SHOOTING);
  }

  public Command getNewSetShooterSpeedCommand(DoubleSupplier speed) {
    return new InstantCommand(
        () -> {
          setShooterSpeed(RPM.of(speed.getAsDouble()));
        },
        this);
  }

  @Override
  public Trigger isSpunUp() {
    return new Trigger(
            () ->
                logged.shooterAngularVelocity.gte(
                    logged.shooterSetpoint.times(spunUpThreshold.get())))
        .debounce(spunUpDebounce.get());
  }
}
