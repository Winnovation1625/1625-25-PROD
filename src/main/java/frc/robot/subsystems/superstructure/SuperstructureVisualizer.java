package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.util.GeomUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class SuperstructureVisualizer {
  private String name;

  public SuperstructureVisualizer(String name) {
    this.name = name;
  }

  private Pose3d elevatorStage1 = new Pose3d();
  private Pose3d elevatorStage2 = new Pose3d();
  private Pose3d elevatorCarriage = new Pose3d();
  private Pose3d arm = new Pose3d();
  private Pose3d wrist = new Pose3d();

  private Pose3d carriagePose;
  private Pose3d armPose;

  public void updateElevatorPose(double distanceFromGround) {
    double carriageDistanceFromGround =
        distanceFromGround - ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS;
    double stage2DistanceFromGround =
        carriageDistanceFromGround - ElevatorConstants.ELEVATOR_CARRIAGE_TRAVEL_DISTANCE;
    double stage1DistanceFromGround =
        stage2DistanceFromGround - ElevatorConstants.ELEVATOR_SECOND_STAGE_TRAVEL_DISTANCE;
    carriagePose =
        elevatorCarriage.transformBy(
            new Transform3d(0, 0, carriageDistanceFromGround, new Rotation3d()));
    Logger.recordOutput("SuperstructureVisualizer/" + name + "/elevatorCarriage", carriagePose);
    Logger.recordOutput(
        "SuperstructureVisualizer/" + name + "/elevatorStage2",
        elevatorStage2.transformBy(
            new Transform3d(
                0,
                0,
                stage2DistanceFromGround > 0 ? stage2DistanceFromGround : 0,
                new Rotation3d())));
    Logger.recordOutput(
        "SuperstructureVisualizer/" + name + "/elevatorStage1",
        elevatorStage1.transformBy(
            new Transform3d(
                0,
                0,
                stage1DistanceFromGround > 0 ? stage1DistanceFromGround : 0,
                new Rotation3d())));
  }

  public void updateArmPose(double armAngleRad) {
    armPose =
        carriagePose.rotateAround(
            carriagePose
                .getTranslation()
                .plus(new Translation3d(0, 0, ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS)),
            new Rotation3d(armAngleRad, 0, 0));
    Logger.recordOutput("SuperstructureVisualizer/" + name + "/arm", armPose);
  }

  //   public void updateWristPose(double wristAngleRad) {
  //     Pose3d wristTemp =
  //         armPose.rotateAround(
  //             armPose
  //                 .getTranslation()
  //                 .plus(
  //                     new Translation3d(0, Units.inchesToMeters(-12.5), Units.inchesToMeters(12))
  //                         .rotateBy(armPose.getRotation())),
  //             new Rotation3d(wristAngleRad, 0, 0));
  //     Logger.recordOutput("SuperstructureVisualizer/" + name + "/wrist", wristTemp);
  //   }

  public void updateSuperstructurePose(double distanceFromGround, double armAngleRad) {
    updateElevatorPose(distanceFromGround);
    updateArmPose(armAngleRad);
  }
}
