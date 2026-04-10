package frc.robot.aiming;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.util.LoggedTunableNumber;

public final class AimingConstants {

  private AimingConstants() {}

  // ===== SHOOTER GEOMETRY (measured) =====
  // Offset of shooter pivot from robot center, robot-frame (x=forward, y=left)
  public static final Translation2d SHOOTER_OFFSET_FROM_CENTER = new Translation2d(0.0, 0.0);

  // Height of the shooter pivot above the ground
  public static final double SHOOTER_PIVOT_HEIGHT_METERS = Units.inchesToMeters(24.0);

  // Robot rotation angular limits (degrees) - full [-180, 180] since robot can face any direction
  public static final double ROBOT_AIM_MIN_DEG = -180.0;
  public static final double ROBOT_AIM_MAX_DEG = 180.0;

  // ===== BALL PHYSICS (tunable for field calibration) =====
  public static final LoggedTunableNumber BALL_MASS_KG =
      new LoggedTunableNumber("Aiming/ballMassKg", 0.27);
  public static final LoggedTunableNumber BALL_RADIUS_M =
      new LoggedTunableNumber("Aiming/ballRadiusM", 0.0508);
  public static final LoggedTunableNumber DRAG_COEFFICIENT =
      new LoggedTunableNumber("Aiming/dragCoefficient", 0.47);
  public static final LoggedTunableNumber LIFT_COEFFICIENT =
      new LoggedTunableNumber("Aiming/liftCoefficient", 0.15);
  public static final double AIR_DENSITY_KG_M3 = 1.225;
  public static final double GRAVITY = 9.81;

  // ===== SHOOTER PARAMETERS (tunable) =====
  public static final LoggedTunableNumber FLYWHEEL_RADIUS_M =
      new LoggedTunableNumber("Aiming/flywheelRadiusM", Units.inchesToMeters(1.5));
  public static final LoggedTunableNumber SPEED_TRANSFER_RATIO =
      new LoggedTunableNumber("Aiming/speedTransferRatio", 0.63);
  public static final double SHOOTER_MIN_RPM = 1000.0;
  public static final double SHOOTER_MAX_RPM = 6000.0;

  // ===== HOOD PARAMETERS =====
  // Hood measures from vertical (0° = straight up), but the aiming algorithm uses
  // 0° = horizontal. Converted: 10° off vertical = 80°, 43° off vertical = 47°.
  // This is correct, it is the ball trajectory & is SUPPOSED to be inverted
  // The hood MAXANGLE should be used to calcute the HOOD_MIN_DEG.
  // The hood MINANGLE should be used to calcute the HOOD_MAX_DEG.
  public static final double HOOD_MIN_DEG = 90.0 - HoodIOTalonFX.MAXANGLE;
  public static final double HOOD_MAX_DEG = 90.0 - HoodIOTalonFX.MINANGLE;

  // ===== SIMULATION PARAMETERS =====
  public static final double SIM_DT = 0.005;
  public static final double SIM_MAX_TIME = 3.0;
  public static final int TRAJECTORY_MAX_POINTS = 100;

  // ===== LAUNCH ANGLE =====
  // Desired field-frame launch angle (degrees). Higher = more lob, must be within hood range.
  // Ball arrives descending as long as this is above the minimum-energy angle (~45° for level
  // targets).
  public static final LoggedTunableNumber TARGET_LAUNCH_ANGLE_DEG =
      new LoggedTunableNumber("Aiming/targetLaunchAngleDeg", 71.0);
  public static final LoggedTunableNumber TARGET_FAR_LAUNCH_ANGLE_DEG =
      new LoggedTunableNumber("Aiming/targetFarLaunchAngleDeg", 65.0);
  public static final LoggedTunableNumber FAR_DISTANCE_THRESHOLD_M =
      new LoggedTunableNumber("Aiming/farDistanceThresholdM", 3.5);
  public static final LoggedTunableNumber FAR_DISTANCE_HYSTERESIS_M =
      new LoggedTunableNumber("Aiming/farDistanceHysteresisM", 0.3);
  public static final LoggedTunableNumber TARGET_PASS_LAUNCH_ANGLE_DEG =
      new LoggedTunableNumber("Aiming/targetPassLaunchAngleDeg", 61.0);

  // ===== PASS TARGET POSITIONS =====
  public static final Translation2d LOW_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(0.9));
  public static final Translation2d HIGH_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(7.1));
  public static final Translation2d LOW_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(0.9));
  public static final Translation2d HIGH_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(7.1));

  // ===== DISTANCE-BASED RPM SCALING =====
  // Adjusts RPM based on distance: rpm *= 1.0 + rpmDistanceScale * (distance - rpmRefDistance)
  // Positive scale → far shots boosted, close shots reduced
  public static final LoggedTunableNumber RPM_DISTANCE_SCALE =
      new LoggedTunableNumber("Aiming/rpmDistanceScale", 0.01);
  public static final LoggedTunableNumber RPM_REF_DISTANCE_M =
      new LoggedTunableNumber("Aiming/rpmRefDistanceM", 5.0);

  // ===== VELOCITY COMPENSATION =====
  // Minimum ball radial speed before solution is considered invalid (m/s)
  public static final double MIN_BALL_RADIAL_SPEED = 0.5;

  // ===== HIGH-FREQUENCY LOOP =====
  // Frequency for aiming computation and motor command threads (Hz)
  public static final double AIMING_FREQUENCY = 250.0;
}
