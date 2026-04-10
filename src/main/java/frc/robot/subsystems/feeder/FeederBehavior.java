package frc.robot.subsystems.feeder;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class FeederBehavior extends SubsystemBehavior {
  private final FeederSubsystem feeder;

  public FeederBehavior(FeederSubsystem feeder) {
    this.feeder = feeder;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isIdleTrigger().whileTrue(feeder.idleCommand());

    events
        .goals()
        .isShootingTrigger()
        .and(events.shooter().isSpunUp())
        .whileTrue(feeder.feedingCommand())
        .whileFalse(feeder.idleCommand());
    events
        .goals()
        .isIntakingTrigger()
        .whileTrue(feeder.intakingCommand())
        .whileFalse(feeder.idleCommand());
  }
}
