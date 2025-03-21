package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants.HumanStation;
import frc.robot.subsystem.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveToHumanStation extends DriveToPose {
  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber(
          "DriveToHumanStation/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber(
          "DriveToHumanStation/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber reefPoseOffsetX =
      new LoggedTunableNumber(
          "DriveToHumanStation/ReefCoralScoreXOffset", Units.inchesToMeters(20));
  private static final LoggedTunableNumber reefPoseOffsetY =
      new LoggedTunableNumber(
          "DriveToHumanStation/ReefCoralScoreYOffset", Units.inchesToMeters(-10.9));

  static {
    minDistanceTagPoseBlend.initDefault(Units.inchesToMeters(24.0));
    maxDistanceTagPoseBlend.initDefault(Units.inchesToMeters(36.0));
    reefPoseOffsetX.initDefault(Units.inchesToMeters(20));
    reefPoseOffsetY.initDefault(Units.inchesToMeters(-10.9));
  }

  public DriveToHumanStation(Drive drive, Supplier<HumanStation> humanStation) {
    super(
        drive,
        // goal supplier
        () ->
            AllianceFlipUtil.apply(
                humanStation
                    .get()
                    .getCenterFace()
                    .transformBy(
                        new Transform2d(
                            reefPoseOffsetX.get(),
                            reefPoseOffsetY.get(),
                            new Rotation2d(Degrees.of(-90))))),
        // robot position supplier
        () -> {
          Optional<Pose2d> txPose =
              drive.getTxTyPose(
                  switch (humanStation.get()) {
                    case LEFT -> AllianceFlipUtil.shouldFlip() ? 2 : 13;
                    case RIGHT -> AllianceFlipUtil.shouldFlip() ? 1 : 12;
                    default -> AllianceFlipUtil.shouldFlip() ? 1 : 12;
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
                                  AllianceFlipUtil.apply(
                                          humanStation
                                              .get()
                                              .getCenterFace()
                                              .transformBy(
                                                  new Transform2d(
                                                      reefPoseOffsetX.get(),
                                                      reefPoseOffsetY.get(),
                                                      new Rotation2d(Degrees.of(-90)))))
                                      .getTranslation())
                          - minDistanceTagPoseBlend.get())
                      / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
                  0.0,
                  1.0);
          return drive.getPose().interpolate(txPose.get(), 1.0 - t);
        });
  }
}
