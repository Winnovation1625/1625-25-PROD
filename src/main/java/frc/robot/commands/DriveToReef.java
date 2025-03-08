package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefPosition;
import frc.robot.subsystem.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

public class DriveToReef extends DriveToPose {
    private final Drive drive;
private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("DriveToReef/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("DriveToReef/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));

    public DriveToReef(Drive drive, ReefPosition reefPosition) {
        super(drive, () -> {
            int face = reefPosition.getFace();
            Pose2d finalPose = FieldConstants.Reef.branchPositions2d.get(reefPosition.getFieldConstantsIndex()).get(0);
            Pose2d currentPose = drive.getPose();
            Optional<Pose2d> txPose = drive.getTxTyPose(switch (face) {
                case 1 -> AllianceFlipUtil.shouldFlip() ? 6 : 19;
                case 2 -> AllianceFlipUtil.shouldFlip() ? 11 : 20;
                case 3 -> AllianceFlipUtil.shouldFlip() ? 10 : 21;
                case 4 -> AllianceFlipUtil.shouldFlip() ? 9 : 22;
                case 5 -> AllianceFlipUtil.shouldFlip() ? 8 : 17;
                default -> AllianceFlipUtil.shouldFlip() ? 7 : 18;
            });
            return getReefPose(face, finalPose, currentPose, txPose);
        }, () -> drive.getPose());
        this.drive = drive;
        ;

    }

public Pose2d getReefPose(int face, Pose2d finalPose, Pose2d currentPose, Optional<Pose2d> txPose) {
    final boolean isRed = AllianceFlipUtil.shouldFlip();
    var tagPose = Optional.of(txPose);
        // drive.getTxTyPose(
        //     switch (face) {
        //       case 1 -> isRed ? 6 : 19;
        //       case 2 -> isRed ? 11 : 20;
        //       case 3 -> isRed ? 10 : 21;
        //       case 4 -> isRed ? 9 : 22;
        //       case 5 -> isRed ? 8 : 17;
        //         // 0
        //       default -> isRed ? 7 : 18;
        //     });
    // Use estimated pose if tag pose is not present
    if (tagPose.isEmpty()) return currentPose;
    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (currentPose.getTranslation().getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend.get())
                / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
            0.0,
            1.0);
    return currentPose.interpolate(tagPose.get(), 1.0 - t);
  }
    
}
