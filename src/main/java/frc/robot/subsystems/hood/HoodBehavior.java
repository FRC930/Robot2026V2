package frc.robot.subsystems.hood;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class HoodBehavior extends SubsystemBehavior {
  private final HoodSubsystem hood;

  public HoodBehavior(HoodSubsystem hood) {
    this.hood = hood;
  }

  @Override
  public void configure(AllEvents events) {
    // events
    //     .goals()
    //     .isIntakingTrigger()
    //     .or(events.goals().isShootingTrigger())
    //     .whileTrue(hood.aimCommand());
    // TODO if idle hood set idle command on false or while false hood aiming command
    events.goals().isShootingTrigger().whileTrue(hood.aimCommand()).whileFalse(hood.idleCommand());
  }
}
