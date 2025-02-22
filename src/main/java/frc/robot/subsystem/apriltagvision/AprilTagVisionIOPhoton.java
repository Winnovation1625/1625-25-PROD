package frc.robot.subsystem.apriltagvision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {

  private final String name;
  private final PhotonCamera camera;
  private int[] tagPoses = new int[] {};

  public AprilTagVisionIOPhoton(String name, Transform3d pose) {

    this.name = name;
    camera = new PhotonCamera(name);
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {

    PhotonPipelineResult result = camera.getLatestResult();
    inputs.isConnected = camera.isConnected();
    if (inputs.isConnected) {
      inputs.numberOfTargets = result.hasTargets() ? result.getTargets().size() : 0;
      inputs.latency = result.metadata.getLatencyMillis();
      inputs.timestamp = result.getTimestampSeconds();
      //   if (result.getMultiTagResult().estimatedPose.isPresent) {
      //     inputs.cameraPoses =
      //         new Pose3d[] {result.getMultiTagResult().estimatedPose.best.toPose3d()};
      //     inputs.ambiguity = result.getMultiTagResult().estimatedPose.ambiguity;
      //     inputs.tagId =
      //         result.getMultiTagResult().fiducialIDsUsed.stream()
      //             .mapToInt(Integer::intValue)
      //             .toArray();
      //     inputs.isMultiTag = true;
      //   } else if (result.hasTargets()) {
      //     inputs.isMultiTag = false;
      //     inputs.cameraPoses =
      //         new Pose3d[] {
      //           result.getBestTarget().getBestCameraToTarget().toPose3d(),
      //           result.getBestTarget().getAlternateCameraToTarget().toPose3d()
      //         };
      //     inputs.ambiguity = result.getBestTarget().getPoseAmbiguity();
      //     inputs.tagId = new int[] {result.getBestTarget().getFiducialId()};
      //   } else {
      //     inputs.isMultiTag = false;
      //     inputs.cameraPoses = new Pose3d[] {};
      //     inputs.ambiguity = -1;
      //     inputs.tagId = tagPoses;
      //   }
      // }

    }
  }
}
