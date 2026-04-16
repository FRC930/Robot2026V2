package frc.robot.aiming;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface AimingEvents {
  Trigger isAimingAtHub();

  Trigger isAimingAtPassLow();

  Trigger isAimingAtPassHigh();

  /** Fires when the robot heading is within tolerance of the commanded aim angle. */
  Trigger isOnTarget();
}
