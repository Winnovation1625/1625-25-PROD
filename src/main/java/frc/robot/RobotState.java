package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.GeomUtil;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class RobotState {

  //   private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  //   static {
  //     for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
  //       tagPoses2d.put(
  //           i,
  //           FieldConstants.defaultAprilTagType
  //               .getLayout()
  //               .getTagPose(i)
  //               .map(Pose3d::toPose2d)
  //               .orElse(new Pose2d()));
  //     }
  // }

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  public record TxTyObservation(
      int tagId, int camera, Pose3d visionPose, double distance, double timestamp) {}

  public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}
}
