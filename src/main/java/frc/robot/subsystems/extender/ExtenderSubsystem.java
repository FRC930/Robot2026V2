package frc.robot.subsystems.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotVisualization;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ExtenderSubsystem extends SubsystemBase implements ExtenderEvents {
  private ExtenderIO m_IO;

  private final EnumState<ExtenderState> m_state =
      new EnumState<>("Extender/States", ExtenderState.IDLE);

  private ExtenderInputsAutoLogged logged = new ExtenderInputsAutoLogged();

  public static final double INCHES_PER_ROT = 4.71;

  public static final double SPOOL_RADIUS = INCHES_PER_ROT / (2.0 * Math.PI);

  public static final double REDUCTION = (50.0 / 15.0);

  private static final LoggedTunableNumber extenderRetractedSetpoint =
      new LoggedTunableNumber("Extender/extenderRetractedSetpoint", 2.0);
  private static final LoggedTunableNumber extenderExtendedSetpoint =
      new LoggedTunableNumber("Extender/extenderExtendedSetpoint", 10.5);
  private static final LoggedTunableNumber extenderIdleSetpoint =
      new LoggedTunableNumber("Extender/extenderIdleSetpoint", 6.0);

  // Motion Magic constraints. Commands set these before issuing a state change, so the motor's
  // trapezoidal profile matches the direction of travel.
  private static final LoggedTunableNumber extendVelocityInchPerSec =
      new LoggedTunableNumber("Extender/extendVelocityInchPerSec", 100.0);
  private static final LoggedTunableNumber extendAccelerationInchPerSec2 =
      new LoggedTunableNumber("Extender/extendAccelerationInchPerSec2", 500.0);
  private static final LoggedTunableNumber retractVelocityInchPerSec =
      new LoggedTunableNumber("Extender/retractVelocityInchPerSec", 25.0);
  private static final LoggedTunableNumber retractAccelerationInchPerSec2 =
      new LoggedTunableNumber("Extender/retractAccelerationInchPerSec2", 500.0);

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder("Gains/Extender/", 50.0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public LoggedTunableGainsBuilder differentialTunableGains =
      new LoggedTunableGainsBuilder("Gains/ExtenderDiff/", 20.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public ExtenderSubsystem(ExtenderIO IO) {
    m_IO = IO;

    logged.distance = Inches.mutable(0);
    logged.followerDistance = Inches.mutable(0);
    logged.differentialPositionError = Inches.mutable(0);
    logged.velocity = InchesPerSecond.mutable(0);
    logged.setPoint = Meters.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.voltage = Volts.mutable(0);

    RobotVisualization.instance().setExtenderExtensionSource(logged.distance);
    // setExtenderHeight(Inches.of(0.0));
  }

  /**
   * @param target
   */
  public void setExtenderHeight(Distance target) {
    m_IO.setExtenderHeight(target);
  }

  public void setTestingState() {
    m_state.set(ExtenderState.TESTING);
  }

  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Extender", logged);

    switch (m_state.get()) {
      case IDLE:
        m_IO.setExtenderHeight(Inches.of(extenderIdleSetpoint.getAsDouble()));
        break;
      case RAISED:
        m_IO.setExtenderHeight(Inches.of(extenderRetractedSetpoint.getAsDouble()));
        break;
      case INTAKING:
      case OUTTAKING:
        m_IO.setExtenderHeight(Inches.of(extenderExtendedSetpoint.getAsDouble()));
        break;
      case TESTING:
        setExtenderHeight(Inches.of(2.0));
        break;
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
    differentialTunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setDifferentialGains(gains));
    Logger.recordOutput("Extender/DifferentialError", logged.differentialPositionError.in(Inches));
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          applyRetractConstraints();
          m_state.set(ExtenderState.IDLE);
        });
  }

  public Command intakeCommand() {
    return runOnce(
        () -> {
          applyExtendConstraints();
          m_state.set(ExtenderState.INTAKING);
        });
  }

  public Command outtakeCommand() {
    return runOnce(
        () -> {
          applyExtendConstraints();
          m_state.set(ExtenderState.OUTTAKING);
        });
  }

  public Command raisedCommand() {
    return runOnce(
        () -> {
          applyRetractConstraints();
          m_state.set(ExtenderState.RAISED);
        });
  }

  /** Retract the extender using the slower "retract" motion profile. */
  public Command retractCommand() {
    return runOnce(
        () -> {
          applyRetractConstraints();
          m_state.set(ExtenderState.RAISED);
        });
  }

  private void applyExtendConstraints() {
    m_IO.setMotionConstraints(extendVelocityInchPerSec.get(), extendAccelerationInchPerSec2.get());
  }

  private void applyRetractConstraints() {
    m_IO.setMotionConstraints(
        retractVelocityInchPerSec.get(), retractAccelerationInchPerSec2.get());
  }

  @Override
  public Trigger idleTrigger() {
    return m_state.is(ExtenderState.IDLE);
  }

  @Override
  public Trigger intakingTrigger() {
    return m_state.is(ExtenderState.INTAKING);
  }

  @Override
  public Trigger outtakingTrigger() {
    return m_state.is(ExtenderState.OUTTAKING);
  }

  @Override
  public Trigger raisedTrigger() {
    return m_state.is(ExtenderState.RAISED);
  }

  public Command getNewDistanceCommand(DoubleSupplier distance) {
    return new InstantCommand(
        () -> {
          m_IO.setExtenderHeight(Inches.of(distance.getAsDouble()));
        },
        this);
  }

  public Command getNewStopCommand() {
    return new InstantCommand(
        () -> {
          m_IO.stop();
        },
        this);
  }
}
