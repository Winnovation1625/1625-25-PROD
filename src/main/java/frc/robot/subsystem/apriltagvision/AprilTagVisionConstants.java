package frc.robot.subsystem.apriltagvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class AprilTagVisionConstants {
  public record CameraConfig(
      int cameraId, String cameraName, Transform3d robotToCamera, double stdDevFactor) {}

  // Basic filtering thresholds
  public static final double MAX_AMBIGUITY = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

  // public static final double ambiguityThreshold = 0.4;
  // public static final double targetLogTimeSecs = 0.1;
  // public static final double fieldBorderMargin = 0.5;
  // public static final double zMargin = 0.75;
  // public static final double xyStdDevCoefficient = 0.015;
  // public static final double thetaStdDevCoefficient = 0.03;
  // public static final double demoTagPosePersistenceSecs = 0.5;
  // public static final double objDetectConfidenceThreshold = 0.8;
  // public static final LoggedTunableNumber timestampOffset =
  //     new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));

  public static final List<CameraConfig> CAMERA_CONFIGS =
      List.of(
          new CameraConfig(
              0,
              "Front Right",
              new Transform3d(
                  Units.inchesToMeters(10.340901),
                  Units.inchesToMeters(-12.659099),
                  Units.inchesToMeters(8),
                  new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-135))),
              1.0),
          new CameraConfig(
              1,
              "Back Right",
              new Transform3d(
                  Units.inchesToMeters(-10.340901),
                  Units.inchesToMeters(-12.659099),
                  Units.inchesToMeters(8),
                  new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-45))),
              1.0),
          new CameraConfig(
              0,
              "Back Left",
              new Transform3d(
                  Units.inchesToMeters(-10.340901),
                  Units.inchesToMeters(12.659099),
                  Units.inchesToMeters(8),
                  new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(45))),
              1.0));
  // new CameraConfig(0, "front left", new Transform3d(Units.inchesToMeters(10.319),
  // Units.inchesToMeters(-12.686), Units.inchesToMeters(8), new Rotation3d(0, 0, 0)), 1.0));
}
