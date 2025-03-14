package frc.robot.subsystem.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVisionConstants.CameraConfig;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class AprilTagVisionIOPhotonSim extends AprilTagVisionIOPhoton {
  private static VisionSystemSim visionSim;

  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public AprilTagVisionIOPhotonSim(
      CameraConfig config,
      Supplier<Rotation2d> headingSupplier,
      Supplier<Pose2d> currentPoseSupplier) {
    super(config, headingSupplier, currentPoseSupplier);

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.defaultAprilTagType.getLayout());
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setFPS(50);
    cameraProperties.setCalibration(1600, 1200, new Rotation2d(Units.degreesToRadians(95)));
    cameraProperties.setLatencyStdDevMs(8);
    cameraProperties.setCalibError(0.35, 0.10);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    visionSim.update(currentPoseSupplier.get());
    super.updateInputs(inputs);
  }
}
