package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

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
}
