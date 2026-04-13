// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  // We are filltering for tags 32,31,30,29,27,26,25,24,21,20,19,18,16,15,14,13,11,10,9,8,5,4,3,2 by
  // using ID filters on the limelights
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String backCamera = "limelight-back";
  public static String leftCamera = "limelight-left";
  public static String rightCamera = "limelight-right";
  public static String questCamName = "Quest";
  /*
   * CAMERA POSITIONS ON ROBOT (looking down from above, with front of robot at top of page)
   *
   * Front Left (x,y)  Front Right (x,y)
   * +/+               +/-
   *
   * Back Left (x,y)   Back Right (x,y)
   * -/+               -/-
   */

  // LL Forward-X: -0.332976, LL Right-Y: 0.038(Negitive in code), LL up-Z: 0.165524, LL Roll 0.0,
  // LL Pitch 150.0, LL Yaw 0.0.
  public static Transform3d robotToBackCamera =
      new Transform3d(
          Units.inchesToMeters(-13.10929),
          Units.inchesToMeters(-1.49063),
          Units.inchesToMeters(6.516693),
          new Rotation3d(0.0, -Units.degreesToRadians(150.0), 0.0));
  // LL Forward-X: -0.01524, LL Right-Y: -0.348198(Negitive in code), LL up-Z: 0.46195, LL Roll 0.0,
  // LL Pitch 0.0, LL Yaw 90.0.
  public static Transform3d robotToLeftCamera =
      new Transform3d(
          Units.inchesToMeters(-0.6),
          Units.inchesToMeters(13.70858),
          Units.inchesToMeters(18.18701),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0)));
  // LL Forward-X: -0.01524, LL Right-Y: 0.348198(Negitive in code), LL up-Z: 0.46195, LL Roll 0.0,
  // LL Pitch 0.0, LL Yaw -90.0.
  public static Transform3d robotToRightCamera =
      new Transform3d(
          Units.inchesToMeters(-0.6),
          Units.inchesToMeters(-13.70858),
          Units.inchesToMeters(18.18701),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.19;
  public static double maxZError = 0.2;
  public static double maxPoseDeltaMeters = 2.0;
  public static double maxAngularVelocityRadPerSec = 5.0;
  public static double maxLatencySeconds = 0.2;
  public static double maxTagDistanceMeters = 4.3;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
