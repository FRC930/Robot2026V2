package frc.robot.aiming;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodIOTalonFX;

public final class AimingConstants {

  private AimingConstants() {}

  // ===== SHOOTER GEOMETRY (measured) =====
  // Offset of shooter pivot from robot center, robot-frame (x=forward, y=left)
  public static final Translation2d SHOOTER_OFFSET_FROM_CENTER =
      new Translation2d(Units.inchesToMeters(-5.93), 0.0);

  // Robot rotation angular limits (degrees) - full [-180, 180] since robot can face any direction
  public static final double ROBOT_AIM_MIN_DEG = -180.0;
  public static final double ROBOT_AIM_MAX_DEG = 180.0;

  public static final double GRAVITY = 9.81;

  // ===== SHOOTER PARAMETERS (tunable) =====
  public static final double FLYWHEEL_RADIUS_M = Units.inchesToMeters(2.0);
  public static final double SPEED_TRANSFER_RATIO = 0.68;
  public static final double SHOOTER_MIN_RPM = 1000.0;
  public static final double SHOOTER_MAX_RPM = 6000.0;

  // ===== HOOD PARAMETERS =====
  // Hood measures from vertical (0° = straight up), but the aiming algorithm uses
  // 0° = horizontal. Converted: 10° off vertical = 80°, 43° off vertical = 47°.
  // This is correct, it is the ball trajectory & is SUPPOSED to be inverted
  // The hood MAXANGLE should be used to calcute the HOOD_MIN_DEG.
  // The hood MINANGLE should be used to calcute the HOOD_MAX_DEG.
  public static final double HOOD_MIN_DEG = HoodIOTalonFX.MINANGLE;
  public static final double HOOD_MAX_DEG = HoodIOTalonFX.MAXANGLE;

  // ===== INTERPOLATION TABLES =====
  // Distance (meters) → hood angle (degrees from horizontal) and shooter RPM.
  // Seed values generated from physics model with updated transfer ratio (0.68).
  // Tune empirically on the real robot.
  public static final InterpolatingDoubleTreeMap HUB_HOOD_ANGLE_MAP =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap HUB_RPM_MAP = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap PASS_HOOD_ANGLE_MAP =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap PASS_RPM_MAP = new InterpolatingDoubleTreeMap();

  static {
    // Hub target: distance (m) → hood angle (deg from horizontal)
    // Seeded at 64° (known good at 7ft). Tune per-distance on robot.
    // HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(3), 64.0);
    HUB_HOOD_ANGLE_MAP.put(2.78, 16.5);
    HUB_HOOD_ANGLE_MAP.put(1.85, 8.0);
    HUB_HOOD_ANGLE_MAP.put(3.8, 24.0);
    HUB_HOOD_ANGLE_MAP.put(3.18, 20.0);
    HUB_HOOD_ANGLE_MAP.put(2.35, 12.0);

    // Hub target: distance (m) → shooter RPM
    // Seed values from physics model, calibrated to 1910 RPM at 7ft
    // HUB_RPM_MAP.put(Units.feetToMeters(3), 2692.0);
    HUB_RPM_MAP.put(2.78, 1895.0);
    HUB_RPM_MAP.put(1.85, 1895.0);
    HUB_RPM_MAP.put(3.8, 1895.0);
    HUB_RPM_MAP.put(3.18, 1895.0);
    HUB_RPM_MAP.put(2.35, 1895.0);

    // Pass target: distance (m) → hood angle (deg from horizontal)
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(10), 30.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(15), 30.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(20), 30.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(25), 30.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(30), 30.0);

    // Pass target: distance (m) → shooter RPM
    PASS_RPM_MAP.put(1.0, 1895.0);
    PASS_RPM_MAP.put(2.0, 1895.0);
    PASS_RPM_MAP.put(3.0, 1895.0);
    PASS_RPM_MAP.put(Units.feetToMeters(10), 2300.0);
    PASS_RPM_MAP.put(Units.feetToMeters(15), 2350.0);
    PASS_RPM_MAP.put(Units.feetToMeters(20), 2400.0);
    PASS_RPM_MAP.put(Units.feetToMeters(25), 2450.0);
    PASS_RPM_MAP.put(Units.feetToMeters(30), 2500.0);
  }

  // ===== PASS TARGET POSITIONS =====
  public static final Translation2d LOW_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(1.1));
  public static final Translation2d HIGH_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(6.9));
  public static final Translation2d LOW_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(1.1));
  public static final Translation2d HIGH_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(6.9));

  // ===== VELOCITY COMPENSATION =====
  // Minimum ball radial speed before solution is considered invalid (m/s)
  public static final double MIN_BALL_RADIAL_SPEED = 0.5;

  // ===== HIGH-FREQUENCY LOOP =====
  // Frequency for aiming computation and motor command threads (Hz)
  public static final double AIMING_FREQUENCY = 250.0;
}
