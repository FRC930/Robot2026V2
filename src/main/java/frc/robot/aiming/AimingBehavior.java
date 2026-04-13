package frc.robot.aiming;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class AimingBehavior extends SubsystemBehavior {

  private final AimingService aimingService;

  public AimingBehavior(AimingService aimingService) {
    this.aimingService = aimingService;
  }

  @Override
  public void configure(AllEvents events) {
    Trigger inNeutralZone = events.drive().isInNeutralZone();
    Trigger onUpperHalf = events.drive().isOnUpperFieldHalf();
    Trigger isOpponentZone = events.drive().isInOpponentZone();

    // Aim at hub when outside the neutral zone
    inNeutralZone
        .negate()
        .and(isOpponentZone.negate())
        .onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.HUB)));

    // In neutral zone, pass to appropriate zone based on field half
    inNeutralZone
        .and(onUpperHalf.negate())
        .onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.PASS_LOW)));

    inNeutralZone
        .and(onUpperHalf)
        .onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.PASS_HIGH)));

    // isOpponentZone
    //     .and(onUpperHalf.negate())
    //     .onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.PASS_LOW)));

    // isOpponentZone
    //     .and(onUpperHalf)
    //     .onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.PASS_HIGH)));
  }
}
