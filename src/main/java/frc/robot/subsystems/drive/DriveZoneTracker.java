package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveZoneTracker extends VirtualSubsystem implements DriveEvents {

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private boolean inNeutralZone = false;
  private boolean onUpperFieldHalf = false;
  private boolean notMoving = false;
  private boolean inOpponentZone = false;

  private static final LoggedTunableNumber notMovingThreshold =
      new LoggedTunableNumber("Drive/notMovingThresholdMps", 0.1);

  private final Trigger inNeutralZoneTrigger = new Trigger(() -> inNeutralZone);
  private final Trigger onUpperFieldHalfTrigger = new Trigger(() -> onUpperFieldHalf);
  private final Trigger notMovingTrigger = new Trigger(() -> notMoving);
  private final Trigger inOpponentTrigger = new Trigger(() -> inOpponentZone);

  public DriveZoneTracker(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
  }

  @Override
  public void periodic() {
    Pose2d pose = poseSupplier.get();
    double x = pose.getX();
    double y = pose.getY();

    inNeutralZone =
        x >= FieldConstants.LinesVertical.neutralZoneNear
            && x <= FieldConstants.LinesVertical.neutralZoneFar;
    onUpperFieldHalf = y > FieldConstants.LinesHorizontal.center;
    inOpponentZone =
        x >= FieldConstants.LinesVertical.oppAllianceZone
            && x <= FieldConstants.LinesVertical.oppAllianceZone;

    ChassisSpeeds speeds = speedsSupplier.get();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    notMoving = linearSpeed < notMovingThreshold.get();

    Logger.recordOutput("DriveZone/InNeutralZone", inNeutralZone);
    Logger.recordOutput("DriveZone/OnUpperFieldHalf", onUpperFieldHalf);
    Logger.recordOutput("DriveZone/NotMoving", notMoving);
    Logger.recordOutput("DriveZone/inOpponentZone", inOpponentZone);
  }

  @Override
  public Trigger isInNeutralZone() {
    return inNeutralZoneTrigger;
  }

  @Override
  public Trigger isOnUpperFieldHalf() {
    return onUpperFieldHalfTrigger;
  }

  @Override
  public Trigger isNotMoving() {
    return notMovingTrigger;
  }

  @Override
  public Trigger isInOpponentZone() {
    return inOpponentTrigger;
  }
}
