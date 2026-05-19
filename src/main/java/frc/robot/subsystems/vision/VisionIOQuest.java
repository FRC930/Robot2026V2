// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** IO implementation for real Limelight hardware. */
public class VisionIOQuest implements VisionIO {
  private Supplier<Pose2d> m_getPose;
  private String cameraName;
  private QuestNav questNav = new QuestNav();
  private boolean m_initialPoseSet = false;
  private boolean m_firstQuestPose = false;
  // TODO: find the pose for quest and rotation
  private final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          Units.inchesToMeters(-10.5),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(25.5),
          new Rotation3d(Rotation2d.fromDegrees(180.0)));
  private int questDebug = 0;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOQuest(Supplier<Pose2d> getPose, String name) {
    this.cameraName = name;
    m_getPose = getPose;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraName = this.cameraName;

    questNav.commandPeriodic(); // Process command responses
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = questNav.isConnected();
    inputs.isQuestPose = true;

    if (questDebug++ > 50) {
      questDebug = 0;
    }

    if (!m_initialPoseSet
        && inputs.connected
        && m_getPose.get() != null
        && m_getPose.get().getX() > 0.0
        && m_getPose.get().getY() > 0.0
        && DriverStation.isEnabled()) {
      // Clear out all unreadFrames() Assume bad given never setPose()
      questNav.getAllUnreadPoseFrames();

      Pose2d currentPose = m_getPose.get();
      questNav.setPose(new Pose3d(currentPose).transformBy(ROBOT_TO_QUEST));
      m_initialPoseSet = true;
      Logger.recordOutput("currentPose", new Pose3d(currentPose).transformBy(ROBOT_TO_QUEST));
    }
    // Monitor connection and device status
    if (questNav.isConnected() && questNav.isTracking() && m_initialPoseSet) {
      List<PoseObservation> poseObservations = new LinkedList<>();
      // System.out.println("Tracking quest");

      // Get latest pose data
      PoseFrame[] newFrames = questNav.getAllUnreadPoseFrames();
      for (PoseFrame questFrame : newFrames) {
        // Use frame.questPose() and frame.dataTimestamp() with pose estimator

        // Get the pose of the Quest
        // Pose3d questPose = questFrame.questPose();
        Pose3d questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        poseObservations.add(
            new PoseObservation(
                // Timestamp, based on server timestamp of publish and latency
                timestamp,
                // 3D pose estimate
                questPose.transformBy(ROBOT_TO_QUEST.inverse()),
                // Ambiguity, using only the first tag because ambiguity isn't applicable for
                // multitag
                0.0,
                // Tag count
                0,
                // Average tag distance
                0.0,
                // Observation type
                PoseObservationType.QUEST));
      }

      // Save pose observations to inputs object
      inputs.poseObservations = new PoseObservation[poseObservations.size()];
      for (int i = 0; i < poseObservations.size(); i++) {
        inputs.poseObservations[i] = poseObservations.get(i);
      }
      if (!m_firstQuestPose && poseObservations.size() > 0) {
        Logger.recordOutput("QuestPose", poseObservations.get(poseObservations.size() - 1).pose());
        m_firstQuestPose = true;
      }
    }
  }
}
