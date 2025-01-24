package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {

  @AutoLog
  class VisionIOInputs {

    public int numberOfTargets;
    public double timestamp;
    public double latency;
    public Pose3d[] cameraPoses = new Pose3d[] {};
    public double ambiguity = -1.0;
    public int[] tagId = new int[] {};
    public boolean isMultiTag;
    public boolean isConnected;
  }

  default String getName() {
    return "";
  }

  default void updateInputs(VisionIOInputs inputs) {}
  ;
}
