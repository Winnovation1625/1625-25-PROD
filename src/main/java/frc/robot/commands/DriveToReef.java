package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.FieldConstants.ReefPosition;
import frc.robot.subsystem.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;

public class DriveToReef extends DriveToPose {
  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("DriveToReef/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("DriveToReef/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));

  public DriveToReef(Drive drive, ReefPosition reefPosition) {
    super(
        drive,
        // goal supplier
        () ->
            FieldConstants.Reef.branchPositions2d.get(reefPosition.getFieldConstantsIndex()).get(ReefLevel.L4),
        // robot position supplier
        () -> {
          Optional<Pose2d> txPose =
              drive.getTxTyPose(
                  switch (reefPosition.getFace()) {
                    case 1 -> AllianceFlipUtil.shouldFlip() ? 6 : 19;
                    case 2 -> AllianceFlipUtil.shouldFlip() ? 11 : 20;
                    case 3 -> AllianceFlipUtil.shouldFlip() ? 10 : 21;
                    case 4 -> AllianceFlipUtil.shouldFlip() ? 9 : 22;
                    case 5 -> AllianceFlipUtil.shouldFlip() ? 8 : 17;
                    default -> AllianceFlipUtil.shouldFlip() ? 7 : 18;
                  });
          if (!txPose.isPresent()) {
            return drive.getPose();
          }
          double t =
              MathUtil.clamp(
                  (drive
                              .getPose()
                              .getTranslation()
                              .getDistance(
                                  FieldConstants.Reef.branchPositions2d
                                      .get(reefPosition.getFieldConstantsIndex())
                                      .get(ReefLevel.L4)
                                      .getTranslation())
                          - minDistanceTagPoseBlend.get())
                      / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
                  0.0,
                  1.0);
          return drive.getPose().interpolate(txPose.get(), 1.0 - t);
        });
  }
}
