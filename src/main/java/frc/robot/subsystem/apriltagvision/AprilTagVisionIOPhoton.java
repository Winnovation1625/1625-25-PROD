package frc.robot.subsystem.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVisionConstants.CameraConfig;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {

  protected final PhotonCamera camera;
  protected final PhotonPoseEstimator multiTagPnp;
  protected final PhotonPoseEstimator txtyPoseEstimator;
  protected final CameraConfig config;
  protected Supplier<Rotation2d> headingSupplier;
  protected Supplier<Pose2d> currentPoseSupplier;

  public AprilTagVisionIOPhoton(
      CameraConfig config,
      Supplier<Rotation2d> headingSupplier,
      Supplier<Pose2d> currentPoseSupplier) {

    this.config = config;
    this.headingSupplier = headingSupplier;
    this.currentPoseSupplier = currentPoseSupplier;
    camera = new PhotonCamera(config.cameraName());
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
    multiTagPnp =
        new PhotonPoseEstimator(
            FieldConstants.defaultAprilTagType.getLayout(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());
    multiTagPnp.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    txtyPoseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.defaultAprilTagType.getLayout(),
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            config.robotToCamera());
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      // if (result.hasTargets()) {
      //   inputs.latestTargetObservation =
      //       new TargetObservation(
      //           Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
      //           Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      // } else {
      //   inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new
      // Rotation2d());
      // }
      multiTagPnp.setReferencePose(currentPoseSupplier.get());
      var tag = multiTagPnp.update(result);
      if (tag.isPresent()) {
        var tagResult = tag.get();

        // Calculate average tag distance

        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(
            tagResult.targetsUsed.stream().map(target -> (short) target.fiducialId).toList());

        // Add pose observation
        if (result.multitagResult.isPresent()) { // Multitag result
          var multitagResult = result.multitagResult.get();

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  tagResult.estimatedPose, // 3D pose estimate
                  multitagResult.estimatedPose.ambiguity, // Ambiguity
                  multitagResult.fiducialIDsUsed.size(), // Tag count
                  totalTagDistance / result.targets.size(), // Average tag distance
                  PoseObservationType.PHOTONVISION_MULTI_TAG)); // Observation type

        } else if (!result.targets.isEmpty()) { // Single tag result
          var target = result.targets.get(0);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  tagResult.estimatedPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  target.bestCameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION_SINGLE_TAG)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
