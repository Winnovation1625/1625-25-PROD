package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.*;
import java.util.*;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
//import frc.robot.RobotState.VisionObservation;

@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends SubsystemBase {

  private final AprilTagVisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private final Alert[] cameraDisconnects =
      new Alert[] {
        new Alert("Front Left Cam disconnected!", Alert.AlertType.WARNING),
        new Alert("Front Right Cam disconnected!", Alert.AlertType.WARNING),
        new Alert("Back Left Cam disconnected!", Alert.AlertType.WARNING),
        new Alert("Back Right Cam disconnected!", Alert.AlertType.WARNING)
      };

  public AprilTagVision(AprilTagVisionIO[] io) {
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }
    // Create map of last detection times for tags
    // FieldConstants.aprilTags
    //     .getTags()
    //     .forEach(
    //         (AprilTag tag) -> {
    //           lastTagDetectionTimes.put(tag.ID, 0.0);
    //         });
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
    }

    // List<Pose3d> allRobotPoses3d = new ArrayList<>();
    // List<VisionObservation> allVisionObservations = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
    //   cameraDisconnects[instanceIndex].set(!inputs[instanceIndex].isConnected);
    //   // Loop over frames
    //   lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
    //   var timestamp = inputs[instanceIndex].timestamp;
      var values = inputs[instanceIndex].cameraPoses;

      if (values.length == 0 || values[0].equals(new Pose3d())) {
        continue;
      }

      Pose3d cameraPose = null;
      Pose3d robotPose3d = null;
      boolean useVisionRotation = false;
      if (inputs[instanceIndex].isMultiTag) {
        cameraPose = inputs[instanceIndex].cameraPoses[0];
        //robotPose3d = cameraPose.plus(cameraLocations[instanceIndex].inverse());
        useVisionRotation = true;
      } else {
        if (inputs[instanceIndex].cameraPoses.length < 2
            || inputs[instanceIndex].cameraPoses[0].equals(new Pose3d())
            || inputs[instanceIndex].cameraPoses[1].equals(new Pose3d())) {
          continue;
        }

        // Pose3d cameraPose0 = inputs[instanceIndex].cameraPoses[0];
        // Pose3d fieldToCamPose0 =
        //     FieldConstants.aprilTags
        //         .getTagPose(inputs[instanceIndex].tagId[0])
        //         .get()
        //         .transformBy(cameraPose0.toTransform3d().inverse());
        // Pose3d cameraPose1 = inputs[instanceIndex].cameraPoses[1];
        // Pose3d fieldToCamPose1 =
        //     FieldConstants.aprilTags
        //         .getTagPose(inputs[instanceIndex].tagId[0])
        //         .get()
        //         .transformBy(cameraPose1.toTransform3d().inverse());
        // Pose3d robotPose3d0 = fieldToCamPose0.plus(cameraLocations[instanceIndex].inverse());
        // Pose3d robotPose3d1 = fieldToCamPose1.plus(cameraLocations[instanceIndex].inverse());

        //  if (inputs[instanceIndex].ambiguity < ambiguityThreshold) {
        //   Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        //   Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
        //   Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
        //   if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
        //       < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
        //     cameraPose = inputs[instanceIndex].cameraPoses[0];
        //     robotPose3d = robotPose3d0;
        //   } else {
        //     cameraPose = inputs[instanceIndex].cameraPoses[1];
        //     robotPose3d = robotPose3d1;
        //   }
      
      }
    }

    

  }
}
