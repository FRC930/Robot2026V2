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
    events.goals().isShootingTrigger().whileTrue(hood.aimCommand()).whileFalse(hood.idleCommand());
  }
}
