package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.AllEvents;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SubsystemBehavior;
import java.util.Set;

public class ExtenderBehavior extends SubsystemBehavior {
  // How long to keep the extender out after the shoot goal engages before retracting.
  private static final LoggedTunableNumber shootRetractDelaySec =
      new LoggedTunableNumber("Extender/shootRetractDelaySec", 0.3);
  private static final LoggedTunableNumber shootExtendDelaySec =
      new LoggedTunableNumber("Extender/shootExtendDelaySec", 0.3);

  private final ExtenderSubsystem extender;

  public ExtenderBehavior(ExtenderSubsystem extender) {
    this.extender = extender;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isOuttakingTrigger().whileTrue(this.extender.outtakeCommand());
    events
        .goals()
        .isIntakingTrigger()
        .whileTrue(this.extender.intakeCommand())
        .whileFalse(this.extender.idleCommand());
    // On shoot: wait, then retract slowly. `Commands.defer` re-reads the tunable on each fire so
    // live-tuning the delay takes effect without a reboot. If the shoot goal drops mid-wait the
    // sequence cancels and the other triggers (intake, idle, etc.) take back over naturally.
    events
        .goals()
        .isShootingTrigger()
        .whileTrue(
            Commands.repeatingSequence(
                this.extender.intakeCommand(),
                Commands.defer(() -> Commands.waitSeconds(shootRetractDelaySec.get()), Set.of()),
                this.extender.retractCommand(),
                Commands.defer(() -> Commands.waitSeconds(shootExtendDelaySec.get()), Set.of())));

    events.goals().isRaisedIntakeTrigger().whileTrue(this.extender.raisedCommand());
  }
}
