// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Double> angularVelocitySupplier;

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<Double> angularVelocitySupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    this.angularVelocitySupplier = angularVelocitySupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera/" + inputs[i].cameraName, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
      boolean isQuestPose = inputs[cameraIndex].isQuestPose;

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose = false;
        if (!isQuestPose) {
          // Subclass rejection (e.g. AprilTagVision rejects MT1 during auto)
          rejectPose = rejectPose(observation);

          // Must have at least one tag
          rejectPose = rejectPose || observation.tagCount() == 0;

          // Reject single-tag with high ambiguity
          rejectPose =
              rejectPose || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity);

          // Must have realistic Z coordinate
          rejectPose = rejectPose || Math.abs(observation.pose().getZ()) > maxZError;

          // Must be within the field boundaries
          rejectPose =
              rejectPose
                  || observation.pose().getX() < 0.0
                  || observation.pose().getX() > aprilTagLayout.getFieldLength()
                  || observation.pose().getY() < 0.0
                  || observation.pose().getY() > aprilTagLayout.getFieldWidth();

          // Reject stale observations
          double latency = Timer.getFPGATimestamp() - observation.timestamp();
          rejectPose = rejectPose || latency > maxLatencySeconds;

          // Reject during fast rotation
          rejectPose =
              rejectPose || Math.abs(angularVelocitySupplier.get()) > maxAngularVelocityRadPerSec;

          // Reject if vision pose is too far from current estimate
          double poseDelta =
              poseSupplier
                  .get()
                  .getTranslation()
                  .getDistance(observation.pose().toPose2d().getTranslation());
          rejectPose = rejectPose || poseDelta > maxPoseDeltaMeters;

          // Reject distant single-tag observations
          rejectPose =
              rejectPose
                  || (observation.tagCount() == 1
                      && observation.averageTagDistance() > maxTagDistanceMeters);
        } else {
          rejectPose =
              // Must be within the field boundaries
              observation.pose().getX() < 0.0
                  || observation.pose().getX() > aprilTagLayout.getFieldLength()
                  || observation.pose().getY() < 0.0
                  || observation.pose().getY() > aprilTagLayout.getFieldWidth();
        }
        // Add pose to log
        robotPoses.add(observation.pose());
        if (!rejectPose) {
          robotPosesAccepted.add(observation.pose());
        } else {
          robotPosesRejected.add(observation.pose());
        }

        // Skip if rejected and not a quest pose
        if (rejectPose) {
          continue;
        }

        if (isQuestPose) {
          addVisionMeasurement(
              observation.pose().toPose2d(),
              observation.timestamp(),
              inputs[cameraIndex].QUESTNAV_STD_DEVS,
              true);
        } else {
          // Calculate standard deviations
          double stdDevFactor =
              Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
          double linearStdDev = linearStdDevBaseline * stdDevFactor;
          double angularStdDev = angularStdDevBaseline * stdDevFactor;
          if (observation.type() == PoseObservationType.MEGATAG_2) {
            linearStdDev *= linearStdDevMegatag2Factor;
            angularStdDev *= angularStdDevMegatag2Factor;
          }
          if (cameraIndex < cameraStdDevFactors.length) {
            linearStdDev *= cameraStdDevFactors[cameraIndex];
            angularStdDev *= cameraStdDevFactors[cameraIndex];
          }
          // send vision observation

          // Send vision observation
          addVisionMeasurement(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
              false);
        }
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera/" + inputs[cameraIndex].cameraName + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera/" + inputs[cameraIndex].cameraName + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera/" + inputs[cameraIndex].cameraName + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera/" + inputs[cameraIndex].cameraName + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs,
        boolean isQuest);
  }

  // accepts the observation
  public boolean rejectPose(PoseObservation observation) {
    return false;
  }

  // accepts the vision measurements
  public void addVisionMeasurement(
      Pose2d pose, double timestamp, Vector<N3> fill, boolean isQuest) {
    consumer.accept(pose, timestamp, fill, isQuest);
  }
}
