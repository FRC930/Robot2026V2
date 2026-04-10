package frc.robot.subsystems.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotVisualization;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ExtenderSubsystem extends SubsystemBase implements ExtenderEvents {
  private ExtenderIO m_IO;

  private final EnumState<ExtenderState> m_state =
      new EnumState<>("Extender/States", ExtenderState.IDLE);

  private ExtenderInputsAutoLogged logged = new ExtenderInputsAutoLogged();

  public static final double INCHES_PER_ROT = 4.71;

  public static final double SPOOL_RADIUS = INCHES_PER_ROT / (2.0 * Math.PI);

  public static final double REDUCTION = (6.00 / 1.0);

  // Agitation state
  private TrapezoidProfile agitateProfile;
  private TrapezoidProfile.State agitateCurrentState = new TrapezoidProfile.State(0, 0);
  private double agitateGoalPosition;
  private boolean agitateInitialized = false;

  // Agitation tunables
  private static final LoggedTunableNumber agitateInDistance =
      new LoggedTunableNumber("Intake/agitateInDistance", 3.0);
  private static final LoggedTunableNumber agitateOutDistance =
      new LoggedTunableNumber("Intake/agitateOutDistance", 8.0);
  private static final LoggedTunableNumber agitateMaxVelocity =
      new LoggedTunableNumber("Intake/agitateMaxVelocityInchPerSec", 20.0);
  private static final LoggedTunableNumber agitateMaxAcceleration =
      new LoggedTunableNumber("Intake/agitateMaxAccelInchPerSec2", 25.0);
  private static final double AGITATE_POSITION_TOLERANCE_DEG = 2.0;

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder("Gains/Extender/", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public LoggedTunableGainsBuilder differentialTunableGains =
      new LoggedTunableGainsBuilder("Gains/ExtenderDiff/", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

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
        break;
      case INTAKING:
        m_IO.setExtenderHeight(Inches.of(12));
        break;
      case OUTTAKING:
        m_IO.setExtenderHeight(Inches.of(12));
        break;
      case SHOOTING:
        m_IO.setExtenderHeight(Inches.of(12));
        break;
      case RAISED:
        m_IO.setExtenderHeight(Inches.of(0));
        break;
      case AGITATING:
        updateAgitation();
        break;
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
    differentialTunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setDifferentialGains(gains));
    Logger.recordOutput("Extender/DifferentialError", logged.differentialPositionError.in(Inches));
  }

  private void updateAgitation() {
    if (!agitateInitialized) {
      agitateProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  agitateMaxVelocity.get(), agitateMaxAcceleration.get()));
      agitateCurrentState = new TrapezoidProfile.State(logged.distance.in(Inches), 0);
      agitateGoalPosition = agitateInDistance.get();
      agitateInitialized = true;
    }

    TrapezoidProfile.State goal = new TrapezoidProfile.State(agitateGoalPosition, 0);
    agitateCurrentState = agitateProfile.calculate(0.020, agitateCurrentState, goal);

    m_IO.setExtenderHeight(Inches.of(agitateCurrentState.position));

    if (Math.abs(agitateCurrentState.position - agitateGoalPosition)
        < AGITATE_POSITION_TOLERANCE_DEG) {
      if (agitateGoalPosition == agitateInDistance.get()) {
        agitateGoalPosition = agitateOutDistance.get();
      } else {
        agitateGoalPosition = agitateInDistance.get();
      }
    }

    Logger.recordOutput("Extender/AgitateSetpoint", agitateCurrentState.position);
    Logger.recordOutput("Extender/AgitateGoal", agitateGoalPosition);
  }

  public Command idleCommand() {
    return runOnce(() -> m_state.set(ExtenderState.IDLE));
  }

  public Command intakeCommand() {
    return runOnce(() -> m_state.set(ExtenderState.INTAKING));
  }

  public Command outtakeCommand() {
    return runOnce(() -> m_state.set(ExtenderState.OUTTAKING));
  }

  public Command shootingCommand() {
    return runOnce(() -> m_state.set(ExtenderState.SHOOTING));
  }

  public Command raisedCommand() {
    return runOnce(() -> m_state.set(ExtenderState.RAISED));
  }

  public Command agitateCommand() {
    return runOnce(() -> m_state.set(ExtenderState.AGITATING));
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
  public Trigger shootingTrigger() {
    return m_state.is(ExtenderState.SHOOTING);
  }

  @Override
  public Trigger raisedTrigger() {
    return m_state.is(ExtenderState.RAISED);
  }

  @Override
  public Trigger agitatingTrigger() {
    return m_state.is(ExtenderState.AGITATING);
  }
}
