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

package frc.robot.subsystem.apriltagvision;

import static frc.robot.subsystem.apriltagvision.AprilTagVisionConstants.ANGULAR_STD_DEV_BASELINE;
import static frc.robot.subsystem.apriltagvision.AprilTagVisionConstants.CAMERA_CONFIGS;
import static frc.robot.subsystem.apriltagvision.AprilTagVisionConstants.LINEAR_STD_DEV_BASELINE;
import static frc.robot.subsystem.apriltagvision.AprilTagVisionConstants.MAX_AMBIGUITY;
import static frc.robot.subsystem.apriltagvision.AprilTagVisionConstants.MAX_Z_ERROR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIO.PoseObservationType;
import frc.robot.util.GeomUtil;
import frc.robot.util.VirtualSubsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends VirtualSubsystem {
  private final VisionConsumer consumer;
  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> poseSupplier;
  // Initialize logging values
  List<Pose3d> allTagPoses = new LinkedList<>();
  List<Pose3d> allRobotPoses = new LinkedList<>();
  List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
  List<Pose3d> allRobotPosesRejected = new LinkedList<>();
  // Initialize logging values
  List<Pose3d> tagPoses = new LinkedList<>();
  List<Pose3d> robotPoses = new LinkedList<>();
  List<Pose3d> robotPosesAccepted = new LinkedList<>();
  List<Pose3d> robotPosesRejected = new LinkedList<>();

  public AprilTagVision(
      VisionConsumer consumer, Supplier<Pose2d> poseSupplier, AprilTagVisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.poseSupplier = poseSupplier;

    // Initialize inputs
    this.inputs = new AprilTagVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagVisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  // /**
  //  * Returns the X angle to the best target, which can be used for simple servoing with vision.
  //  *
  //  * @param cameraIndex The index of the camera to use.
  //  */
  // public Rotation2d getTargetX(int cameraIndex) {
  //   return inputs[cameraIndex].latestTargetObservation.tx();
  // }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + CAMERA_CONFIGS.get(i).cameraName(), inputs[i]);
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > MAX_AMBIGUITY) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX()
                    > FieldConstants.defaultAprilTagType.getLayout().getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY()
                    > FieldConstants.defaultAprilTagType.getLayout().getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }
        if (observation.type() == PoseObservationType.PHOTONVISION_MULTI_TAG
            || observation.type() == PoseObservationType.PHOTONVISION_SINGLE_TAG) {
          // Calculate standard deviations
          double stdDevFactor =
              Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
          double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
          double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
          if (cameraIndex < CAMERA_CONFIGS.size()) {
            linearStdDev *= CAMERA_CONFIGS.get(cameraIndex).stdDevFactor();
            angularStdDev *= CAMERA_CONFIGS.get(cameraIndex).stdDevFactor();
          }

          // Send vision observation
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
              null,
              null);
        }
        if (observation.type() == PoseObservationType.PHOTONVISION_TX_TY) {
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              null,
              observation.averageTagDistance(),
              observation.tagCount());
        }
      }

      // Log camera datadata
      // TODO: Remvove Rejected and Robot Poses from any logging since it includes bad poses, only
      // pass accepted poses to drive, less load on NT
      Logger.recordOutput(
          "Vision/" + CAMERA_CONFIGS.get(cameraIndex).cameraName() + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/" + CAMERA_CONFIGS.get(cameraIndex).cameraName() + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/" + CAMERA_CONFIGS.get(cameraIndex).cameraName() + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/" + CAMERA_CONFIGS.get(cameraIndex).cameraName() + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      Logger.recordOutput(
          "Vision/" + CAMERA_CONFIGS.get(cameraIndex).cameraName() + "/CameraPose",
          new Pose3d(poseSupplier.get())
              .transformBy(CAMERA_CONFIGS.get(cameraIndex).robotToCamera()));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();
  }

  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs,
        Double distance,
        Integer tagId);
  }
}
