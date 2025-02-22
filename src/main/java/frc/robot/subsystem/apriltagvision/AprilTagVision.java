// Copyright (c) 2024 FRC 1625
// https://github.com/Winnovation1625
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystem.apriltagvision;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOInputsAutoLogged;
import frc.robot.util.Alert;
import frc.robot.FieldConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.VirtualSubsystem;

import static frc.robot.subsystem.apriltagvision.VisionConstants.*;

import java.util.*;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends VirtualSubsystem {
  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputsAutoLogged[] inputs;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  //TODO: Update camera locations
  private final Alert[] cameraDisconnects =
      new Alert[] {
        new Alert("Front Left Cam disconnected!", Alert.AlertType.WARNING),
        new Alert("Front Right Cam disconnected!", Alert.AlertType.WARNING),
        new Alert("Back Left Cam disconnected!", Alert.AlertType.WARNING),
        new Alert("Back Right Cam disconnected!", Alert.AlertType.WARNING)
      };

  public AprilTagVision(AprilTagVisionIO[] io) {
    this.io = io;
    inputs = new AprilTagVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputsAutoLogged();
    }
    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }
    // Create map of last detection times for tags
    FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout()
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              lastTagDetectionTimes.put(tag.ID, 0.0);
            });
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
    }

    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      cameraDisconnects[instanceIndex].set(!inputs[instanceIndex].isConnected);
      // Loop over frames
      lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
      var timestamp = inputs[instanceIndex].timestamp;
      var values = inputs[instanceIndex].cameraPoses;

      // Exit if blank frame
      if (values.length == 0 || values[0].equals(new Pose3d())) {
        continue;
      }

      // Switch based on number of poses
      Pose3d cameraPose = null;
      Pose3d robotPose3d = null;
      boolean useVisionRotation = false;
      if (inputs[instanceIndex].isMultiTag) {
        cameraPose = inputs[instanceIndex].cameraPoses[0];
        robotPose3d = null; //TODO Convert this from Translation3d to Pose3d cameraPose.plus(cameraPoses.get(instanceIndex).inverse());
        useVisionRotation = true;
      } else {
        if (inputs[instanceIndex].cameraPoses.length < 2
            || inputs[instanceIndex].cameraPoses[0].equals(new Pose3d())
            || inputs[instanceIndex].cameraPoses[1].equals(new Pose3d())) {
          continue;
        }
        // Two poses (one tag), disambiguate
        Pose3d cameraPose0 = inputs[instanceIndex].cameraPoses[0];
        Pose3d fieldToCamPose0 =
            FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout()
                .getTagPose(inputs[instanceIndex].tagId[0])
                .get()
                .transformBy(cameraPose0.toTransform3d().inverse());
        Pose3d cameraPose1 = inputs[instanceIndex].cameraPoses[1];
        Pose3d fieldToCamPose1 =
            FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout()
                .getTagPose(inputs[instanceIndex].tagId[0])
                .get()
                .transformBy(cameraPose1.toTransform3d().inverse());
        Pose3d robotPose3d0 = null; //TODO Convert this from Translation3d to Pose3d fieldToCamPose0.plus(cameraPoses[instanceIndex].inverse());
        Pose3d robotPose3d1 = null; //TODO Convert this from Translation3d to Pose3d fieldToCamPose1.plus(cameraPoses[instanceIndex].inverse());

        // Check for ambiguity and select based on estimated rotation
        if (inputs[instanceIndex].ambiguity < ambiguityThreshold) {
          Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
          Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
          Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
          if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
              < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
            cameraPose = inputs[instanceIndex].cameraPoses[0];
            robotPose3d = robotPose3d0;
          } else {
            cameraPose = inputs[instanceIndex].cameraPoses[1];
            robotPose3d = robotPose3d1;
          }
        }
      }

      // Exit if no data
      if (cameraPose == null || robotPose3d == null) {
        continue;
      }

      // // Exit if robot pose is off the field
      if (robotPose3d.getX() < -fieldBorderMargin
          || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
          || robotPose3d.getY() < -fieldBorderMargin
          || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
          || robotPose3d.getZ() < -zMargin
          || robotPose3d.getZ() > zMargin) {
        continue;
      }

      // Get 2D robot pose
      Pose2d robotPose = robotPose3d.toPose2d();

      // Get tag poses and update last detection times
      List<Pose3d> tagPoses = new ArrayList<>();
      for (int tagId : inputs[instanceIndex].tagId) {
        lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
        Optional<Pose3d> tagPose = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Calculate average distance to tag
      double totalDistance = 0.0;
      for (Pose3d tagPose : tagPoses) {
        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      }
      double avgDistance;
      if (tagPoses.size() == 0) {
        avgDistance = 1; // one off case where tagId is not returned but estimated Pose is
      } else {
        avgDistance = totalDistance / tagPoses.size();
      }

      // Add observation to list
      double xyStdDev =
          xyStdDevCoefficient
              * Math.pow(avgDistance, 2.0)
              / tagPoses.size()
              * VisionConstants.cameraConfigs.get(instanceIndex).stdDevFactor();
      double thetaStdDev =
          useVisionRotation
              ? thetaStdDevCoefficient
                  * Math.pow(avgDistance, 2.0)
                  / tagPoses.size()
                  * VisionConstants.cameraConfigs.get(instanceIndex).stdDevFactor()
              : Double.POSITIVE_INFINITY;
      allVisionObservations.add(
          new VisionObservation(
              robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
      allRobotPoses3d.add(robotPose3d);

      // Log data from instance
      Logger.recordOutput(
          "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
          Timer.getFPGATimestamp() - timestamp);
      Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
      Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
      Logger.recordOutput(
          "AprilTagVision/Inst" + instanceIndex + "/RobotPose3dRotYaw",
          robotPose3d.getRotation().getZ());
      Logger.recordOutput(
          "AprilTagVision/Inst" + instanceIndex + "/RobotPose3dRotRoll",
          robotPose3d.getRotation().getX());
      Logger.recordOutput(
          "AprilTagVision/Inst" + instanceIndex + "/RobotPose3dRotPitch",
          robotPose3d.getRotation().getY());
      Logger.recordOutput(
          "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));

      // If no frames from instances, clear robot pose
      if (inputs[instanceIndex].numberOfTargets == 0) {
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", new Pose2d());
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", new Pose3d());
      }

      // If no recent frames from instance, clear tag poses
      if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
        //noinspection RedundantArrayCreation
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[] {});
      }
    }
    Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        allTagPoses.add(FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(detectionEntry.getKey()).get());
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

    // Send results to robot state
    allVisionObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState.getInstance()::addVisionObservation);
  }
}