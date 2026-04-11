package frc.robot.subsystems.intake;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class IntakeBehavior extends SubsystemBehavior {

  private final IntakeSubsystem intake;

  public IntakeBehavior(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isIdleTrigger().whileTrue(this.intake.idleCommand());
    events.goals().isOuttakingTrigger().whileTrue(this.intake.outtakeCommand());
    events.goals().isIntakingTrigger().whileTrue(this.intake.intakeCommand());
    events
        .goals()
        .isShootingTrigger()
        .and(events.drive().isNotMoving().negate())
        .whileTrue(this.intake.shootingCommand());
    events
        .goals()
        .isShootingTrigger()
        .and(events.drive().isNotMoving())
        .whileTrue(this.intake.agitateCommand());
    events.goals().isRaisedIntakeTrigger().whileTrue(this.intake.raisedCommand());
    events.goals().isClimbingL0().whileTrue(this.intake.idleCommand());
    events.goals().isClimbingL1().whileTrue(this.intake.idleCommand());
    events.goals().isClimbingL2().whileTrue(this.intake.idleCommand());
    events.goals().isClimbingL3().whileTrue(this.intake.idleCommand());
  }
}
