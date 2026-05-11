package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriveEvents {
  Trigger isInNeutralZone();

  Trigger isInBlueZone();

  Trigger isInRedZone();

  Trigger isOnUpperFieldHalf();

  Trigger isNotMoving();
}
