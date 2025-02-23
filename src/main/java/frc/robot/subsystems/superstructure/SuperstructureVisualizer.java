package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperstructureVisualizer {

  @AutoLogOutput private Pose3d elevatorStage1 = new Pose3d();
  @AutoLogOutput private Pose3d elevatorStage2 = new Pose3d();
  @AutoLogOutput private Pose3d elevatorCarriage = new Pose3d();
  @AutoLogOutput private Pose3d arm = new Pose3d();
  @AutoLogOutput private Pose3d wrist = new Pose3d();

  public void updateElevatorPose(double distanceFromGround) {}

  public void updateArmPose(double armAngleRad) {}

  public void updateWristPose(double wristAngleRad) {}

  public void updateSuperstructurePose(
      double distanceFromGround, double armAngleRad, double wristAngleRad) {
    updateElevatorPose(distanceFromGround);
    updateArmPose(armAngleRad);
    updateWristPose(wristAngleRad);
  }
}
