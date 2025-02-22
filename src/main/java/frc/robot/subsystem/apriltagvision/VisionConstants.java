package frc.robot.subsystem.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;

public class VisionConstants {
  public record CameraConfig(int cameraId, Pose3d cameraToRobot, double stdDevFactor) {}

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.015;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double objDetectConfidenceThreshold = 0.8;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));

  public static final List<CameraConfig> cameraConfigs =
      List.of(new CameraConfig(0, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), 0.1));
}
