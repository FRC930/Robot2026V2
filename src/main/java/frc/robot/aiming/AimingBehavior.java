package frc.robot.aiming;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    Trigger inRedZone = events.drive().isInRedZone();
    Trigger inBlueZone = events.drive().isInBlueZone();
    Trigger onUpperHalf = events.drive().isOnUpperFieldHalf();
    Trigger isPlayingBlue =
        new Trigger(() -> (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue));
    Trigger inOurSide =
        inNeutralZone
            .negate()
            .and(isPlayingBlue.and(inBlueZone).or(isPlayingBlue.negate().and(inRedZone)));

    // Aim at hub when outside the neutral zone
    inOurSide.onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.HUB)));

    // In neutral zone, pass to appropriate zone based on field half
    inOurSide
        .negate()
        .and(onUpperHalf.negate())
        .onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.PASS_LOW)));

    inOurSide
        .negate()
        .and(onUpperHalf)
        .onTrue(Commands.runOnce(() -> aimingService.setTarget(AimingTarget.PASS_HIGH)));
  }
}
