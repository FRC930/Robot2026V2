package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ShooterEvents {
  // These triggers take from shooter state
  public Trigger isIdleTrigger();

  public Trigger isShootingTrigger();

  public Trigger isSpunUp();
}
