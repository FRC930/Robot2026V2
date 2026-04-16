package frc.robot.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.PoseSnapshot;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Central aiming service that computes robot aim angle, hood angle, and shooter RPM. Computation
 * runs at 250Hz via AimingThread; logging and visualization run at 50Hz via periodic().
 */
public class AimingService extends VirtualSubsystem implements AimingEvents {

  private final Supplier<PoseSnapshot> snapshotSupplier;

  private final EnumState<AimingTarget> targetState =
      new EnumState<>("Aiming/Target", AimingTarget.HUB);

  // Thread-safe mirror of targetState (EnumState.currentState is not volatile)
  private volatile AimingTarget currentTarget = AimingTarget.HUB;

  // Volatile outputs written by computeAimingSolution() at 250Hz
  private volatile double aimAngleDeg = 0.0;
  private volatile double hoodAngleDeg = 45.0;
  private volatile double shooterRPM = 0.0;
  private volatile double distanceToTargetM = 0.0;
  private volatile boolean solutionValid = false;

  // Cached launcher params for BallTrajectorySim at 50Hz
  private volatile double cachedLauncherSpeed = 0.0;
  private volatile double cachedLauncherAngleRad = 0.0;

  // EMA smoothing — alpha in (0, 1]. Lower = smoother, 1.0 = no filtering.
  private static final LoggedTunableNumber rpmEmaAlpha =
      new LoggedTunableNumber("Aiming/Smoothing/rpmEmaAlpha", 0.08);
  private static final LoggedTunableNumber aimEmaAlpha =
      new LoggedTunableNumber("Aiming/Smoothing/aimEmaAlpha", 0.15);
  private static final LoggedTunableNumber hoodEmaAlpha =
      new LoggedTunableNumber("Aiming/Smoothing/hoodEmaAlpha", 0.10);

  private static final LoggedTunableNumber negativeAimingCompensation =
      new LoggedTunableNumber("Aiming/negativeAimingCompensation", -30.00);
  private static final LoggedTunableNumber positiveAimingCompensation =
      new LoggedTunableNumber("Aiming/positiveAimingCompensation", 30.00);

  // Tolerance (degrees) for isOnTarget(): how close robot heading must be to aim angle.
  private static final LoggedTunableNumber onTargetToleranceDeg =
      new LoggedTunableNumber("Aiming/onTargetToleranceDeg", 5.0);
  // Debounce (seconds) to avoid flickering when crossing the threshold.
  private static final LoggedTunableNumber onTargetDebounce =
      new LoggedTunableNumber("Aiming/onTargetDebounce", 0.08);

  // EMA state (only written by AimingThread, no synchronization needed)
  private double smoothedRPM = 0.0;
  private double smoothedAimDeg = 0.0;
  private double smoothedHoodDeg = 0.0;
  private boolean emaInitialized = false;

  public static final BallTrajectorySim trajectorySim = new BallTrajectorySim();

  public AimingService(Supplier<PoseSnapshot> snapshotSupplier) {
    this.snapshotSupplier = snapshotSupplier;
  }

  public void setTarget(AimingTarget target) {
    targetState.set(target);
    currentTarget = target;
  }

  @Override
  public Trigger isAimingAtHub() {
    return targetState.is(AimingTarget.HUB);
  }

  @Override
  public Trigger isAimingAtPassLow() {
    return targetState.is(AimingTarget.PASS_LOW);
  }

  @Override
  public Trigger isAimingAtPassHigh() {
    return targetState.is(AimingTarget.PASS_HIGH);
  }

  @Override
  public Trigger isOnTarget() {
    return new Trigger(() -> Math.abs(getHeadingErrorDeg()) <= onTargetToleranceDeg.get())
        .debounce(onTargetDebounce.get());
  }

  /** Wrapped signed error (degrees) between current heading and commanded aim angle. */
  private double getHeadingErrorDeg() {
    double headingDeg = snapshotSupplier.get().heading().getDegrees();
    return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(headingDeg - aimAngleDeg)));
  }

  /** Called at 250Hz by AimingThread. Reads snapshot, extrapolates pose, computes solution. */
  public void computeAimingSolution() {
    PoseSnapshot snapshot = snapshotSupplier.get();

    // Linear extrapolation: estimate current pose using velocity * dt
    double now = Timer.getFPGATimestamp();
    double dt = MathUtil.clamp(now - snapshot.timestampSeconds(), 0.0, 0.1);

    ChassisSpeeds speeds = snapshot.chassisSpeeds();
    Rotation2d heading = snapshot.heading();

    // Field-relative velocity for extrapolation
    double cos = heading.getCos();
    double sin = heading.getSin();
    double vxField = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
    double vyField = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

    Pose2d basePose = snapshot.pose();
    Pose2d robotPose =
        new Pose2d(
            basePose.getX() + vxField * dt,
            basePose.getY() + vyField * dt,
            basePose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
    Rotation2d extrapolatedHeading = robotPose.getRotation();

    // Compute the aiming solution using extrapolated pose
    computeForPose(robotPose, speeds, extrapolatedHeading);
  }

  /** 50Hz logging and visualization only. Reads volatile fields. */
  @Override
  public void periodic() {
    targetState.log();
    logOutputs();

    // Trajectory visualization at 50Hz — skip on real robot to save CPU
    if (Constants.currentMode != Constants.Mode.REAL) {
      PoseSnapshot snapshot = snapshotSupplier.get();
      if (solutionValid) {
        Rotation2d heading = snapshot.heading();
        Translation2d shooterFieldPos = computeShooterFieldPosition(snapshot.pose(), heading);
        double fieldAimYaw = heading.getRadians() + Math.PI + Math.toRadians(aimAngleDeg);
        Translation2d shooterVelocity = computeShooterVelocity(snapshot.chassisSpeeds(), heading);
        trajectorySim.simulate(
            shooterFieldPos,
            fieldAimYaw,
            cachedLauncherAngleRad, // TODO fix on blue side
            cachedLauncherSpeed,
            shooterVelocity);
      } else {
        trajectorySim.publishEmpty();
      }
    }
  }

  private void computeForPose(Pose2d robotPose, ChassisSpeeds speeds, Rotation2d heading) {
    Translation3d target = getTargetPosition();
    Translation2d shooterFieldPos = computeShooterFieldPosition(robotPose, heading);

    // Compute distances to target
    double dx = target.getX() - shooterFieldPos.getX();
    double dy = target.getY() - shooterFieldPos.getY();
    double horizontalDistance = Math.hypot(dx, dy);

    distanceToTargetM = horizontalDistance;

    // Field-frame angle to target, converted to robot aim frame (0° = robot backward)
    double fieldAngleToTargetRad = Math.atan2(dy, dx);
    double aimAngleRad =
        MathUtil.angleModulus(fieldAngleToTargetRad - /*heading.getRadians() -*/ Math.PI);
    double rawAimAngleDeg = Math.toDegrees(aimAngleRad);

    // Velocity compensation: project shooter velocity onto radial and tangential axes
    Translation2d shooterVelocity = computeShooterVelocity(speeds, heading);
    double ux = Math.cos(fieldAngleToTargetRad);
    double uy = Math.sin(fieldAngleToTargetRad);
    double vRadial = shooterVelocity.getX() * ux + shooterVelocity.getY() * uy;
    double vTangential = -shooterVelocity.getX() * uy + shooterVelocity.getY() * ux;

    // Look up hood angle and RPM from distance-based interpolation tables

    double interpolatedHoodDeg;
    double rpm;
    if (currentTarget == AimingTarget.HUB) {
      interpolatedHoodDeg = AimingConstants.HUB_HOOD_ANGLE_MAP.get(horizontalDistance);
      rpm = AimingConstants.HUB_RPM_MAP.get(horizontalDistance);
    } else {
      interpolatedHoodDeg = AimingConstants.PASS_HOOD_ANGLE_MAP.get(horizontalDistance);
      rpm = AimingConstants.PASS_RPM_MAP.get(horizontalDistance);
    }
    Logger.recordOutput("Aiming/interpolatedHoodDeg", interpolatedHoodDeg);

    // Reverse-derive ball speed from RPM for velocity compensation and sim visualization
    double flywheelCircumference = 2.0 * Math.PI * AimingConstants.FLYWHEEL_RADIUS_M;
    double ballSpeed = (rpm * flywheelCircumference / 60.0) * AimingConstants.SPEED_TRANSFER_RATIO;
    double hoodAngleRad = Math.toRadians(interpolatedHoodDeg);
    double ballHorizontal = ballSpeed * Math.cos(hoodAngleRad);

    // Velocity compensation: check radial ball speed exceeds robot motion
    double launcherRadial = ballHorizontal - vRadial;

    if (launcherRadial <= AimingConstants.MIN_BALL_RADIAL_SPEED) {
      solutionValid = false;
      return;
    }

    // Yaw correction: aim off-target to cancel tangential velocity
    double yawCorrectionRad = Math.atan2(-vTangential, launcherRadial);
    double compensatedAimDeg = rawAimAngleDeg + Math.toDegrees(yawCorrectionRad);

    double robotRotation = robotPose.getRotation().getDegrees();

    // Clamp and validate
    boolean aimInRange =
        robotRotation >= aimAngleDeg + negativeAimingCompensation.get()
            && robotRotation <= aimAngleDeg + positiveAimingCompensation.get();
    boolean hoodInRange =
        interpolatedHoodDeg >= AimingConstants.HOOD_MIN_DEG
            && interpolatedHoodDeg <= AimingConstants.HOOD_MAX_DEG;
    boolean rpmInRange =
        rpm >= AimingConstants.SHOOTER_MIN_RPM && rpm <= AimingConstants.SHOOTER_MAX_RPM;

    solutionValid = aimInRange && hoodInRange && rpmInRange;
    Logger.recordOutput("Aiming/compensatedAimDeg", compensatedAimDeg);
    double clampedAim =
        MathUtil.clamp(
            compensatedAimDeg,
            AimingConstants.ROBOT_AIM_MIN_DEG,
            AimingConstants.ROBOT_AIM_MAX_DEG);
    double clampedHood =
        MathUtil.clamp(
            interpolatedHoodDeg, AimingConstants.HOOD_MIN_DEG, AimingConstants.HOOD_MAX_DEG);
    double clampedRPM =
        MathUtil.clamp(rpm, AimingConstants.SHOOTER_MIN_RPM, AimingConstants.SHOOTER_MAX_RPM);

    // EMA smoothing to reduce high-frequency noise from pose estimation
    if (!emaInitialized) {
      smoothedAimDeg = clampedAim;
      smoothedHoodDeg = clampedHood;
      smoothedRPM = clampedRPM;
      emaInitialized = true;
    } else {
      double aT = aimEmaAlpha.get();
      double aH = hoodEmaAlpha.get();
      double aR = rpmEmaAlpha.get();
      smoothedAimDeg = aT * clampedAim + (1.0 - aT) * smoothedAimDeg;
      smoothedHoodDeg = aH * clampedHood + (1.0 - aH) * smoothedHoodDeg;
      smoothedRPM = aR * clampedRPM + (1.0 - aR) * smoothedRPM;
    }

    aimAngleDeg = smoothedAimDeg;
    hoodAngleDeg = smoothedHoodDeg;
    shooterRPM = smoothedRPM;

    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

    // Cache launcher params for 50Hz trajectory visualization
    cachedLauncherSpeed = ballSpeed;
    if (isBlue) {
      cachedLauncherAngleRad = Math.PI - hoodAngleRad;
    } else {
      cachedLauncherAngleRad = hoodAngleRad;
    }
  }

  private Translation3d getTargetPosition() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    switch (currentTarget) {
      case PASS_LOW:
        Translation2d lowPass =
            isRed ? AimingConstants.LOW_RED_PASS : AimingConstants.LOW_BLUE_PASS;
        return new Translation3d(lowPass);
      case PASS_HIGH:
        Translation2d highPass =
            isRed ? AimingConstants.HIGH_RED_PASS : AimingConstants.HIGH_BLUE_PASS;
        return new Translation3d(highPass);
      case HUB:
      default:
        return isRed ? FieldConstants.Hub.oppTopCenterPoint : FieldConstants.Hub.topCenterPoint;
    }
  }

  private Translation2d computeShooterFieldPosition(Pose2d robotPose, Rotation2d heading) {
    Translation2d rotatedOffset = AimingConstants.SHOOTER_OFFSET_FROM_CENTER.rotateBy(heading);
    return robotPose.getTranslation().plus(rotatedOffset);
  }

  private Translation2d computeShooterVelocity(ChassisSpeeds speeds, Rotation2d heading) {
    double cos = heading.getCos();
    double sin = heading.getSin();
    double vxField = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
    double vyField = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

    Translation2d rotatedOffset = AimingConstants.SHOOTER_OFFSET_FROM_CENTER.rotateBy(heading);
    double omega = speeds.omegaRadiansPerSecond;
    double vxRot = -omega * rotatedOffset.getY();
    double vyRot = omega * rotatedOffset.getX();

    return new Translation2d(vxField + vxRot, vyField + vyRot);
  }

  private void logOutputs() {
    Logger.recordOutput("Aiming/AimAngleDeg", aimAngleDeg);
    Logger.recordOutput("Aiming/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Aiming/ShooterRPM", shooterRPM);
    Logger.recordOutput("Aiming/DistanceToTargetM", distanceToTargetM);
    Logger.recordOutput("Aiming/SolutionValid", solutionValid);
    PoseSnapshot snapshot = snapshotSupplier.get();
    Logger.recordOutput("Aiming/RobotPose", snapshot.pose());
    Logger.recordOutput("Aiming/TargetPosition", getTargetPosition());
    Logger.recordOutput("Aiming/HeadingErrorDeg", getHeadingErrorDeg());
  }

  public double getAimAngleDeg() {
    return aimAngleDeg;
  }

  public double getHoodAngleDeg() {
    return hoodAngleDeg;
  }

  public double getShooterRPM() {
    return shooterRPM;
  }

  public boolean isSolutionValid() {
    return solutionValid;
  }
}
