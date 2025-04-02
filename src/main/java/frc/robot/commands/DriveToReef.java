package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AlgaeObjective;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.FieldConstants.ReefPosition;
import frc.robot.subsystem.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveToReef extends DriveToPose {
  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("DriveToReef/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("DriveToReef/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber reefPoseOffsetX =
      new LoggedTunableNumber("DriveToReef/ReefCoralScoreXOffset", Units.inchesToMeters(17));
  private static final LoggedTunableNumber reefPoseOffsetY =
      new LoggedTunableNumber("DriveToReef/ReefCoralScoreYOffset", Units.inchesToMeters(-10.725));

  static {
    minDistanceTagPoseBlend.initDefault(Units.inchesToMeters(24.0));
    maxDistanceTagPoseBlend.initDefault(Units.inchesToMeters(36.0));
    reefPoseOffsetX.initDefault(Units.inchesToMeters(17));
    reefPoseOffsetY.initDefault(Units.inchesToMeters(-10.725));
  }

  public DriveToReef(Drive drive, Supplier<ReefPosition> reefPosition) {
    super(
        drive,
        // goal supplier
        () ->
            AllianceFlipUtil.apply(
                FieldConstants.Reef.branchPositions2d
                    .get(reefPosition.get().getFieldConstantsIndex())
                    .get(ReefLevel.L4)
                    .transformBy(
                        new Transform2d(
                            reefPoseOffsetX.get(),
                            reefPoseOffsetY.get(),
                            new Rotation2d(Degrees.of(-90))))),
        // robot position supplier
        () -> {
          Optional<Pose2d> txPose =
              drive.getTxTyPose(
                  switch (reefPosition.get().getFace()) {
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
                                  AllianceFlipUtil.apply(
                                          FieldConstants.Reef.branchPositions2d
                                              .get(reefPosition.get().getFieldConstantsIndex())
                                              .get(ReefLevel.L4)
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

  public DriveToReef(Drive drive, int reefFace, boolean isCoral) {
    super(
        drive,
        // goal supplier
        () ->
            AllianceFlipUtil.apply(
                FieldConstants.Reef.centerFaces[reefFace].transformBy(
                    new Transform2d(
                        reefPoseOffsetX.get(),
                        isCoral ? reefPoseOffsetY.get() : -reefPoseOffsetY.get(),
                        isCoral
                            ? new Rotation2d(Degrees.of(-90))
                            : new Rotation2d(Degrees.of(90))))),
        // robot position supplier
        () -> {
          Optional<Pose2d> txPose =
              drive.getTxTyPose(
                  switch (reefFace) {
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
                                  AllianceFlipUtil.apply(
                                          FieldConstants.Reef.centerFaces[reefFace].transformBy(
                                              new Transform2d(
                                                  reefPoseOffsetX.get(),
                                                  isCoral
                                                      ? reefPoseOffsetY.get()
                                                      : -reefPoseOffsetY.get(),
                                                  isCoral
                                                      ? new Rotation2d(Degrees.of(-90))
                                                      : new Rotation2d(Degrees.of(90)))))
                                      .getTranslation())
                          - minDistanceTagPoseBlend.get())
                      / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
                  0.0,
                  1.0);
          return drive.getPose().interpolate(txPose.get(), 1.0 - t);
        });
  }

  public DriveToReef(
      Drive drive, Supplier<AlgaeObjective> algaeObjective, Supplier<Boolean> isCoral) {
    super(
        drive,
        // goal supplier
        () ->
            AllianceFlipUtil.apply(
                FieldConstants.Reef.centerFaces[algaeObjective.get().position().getFace()]
                    .transformBy(
                        new Transform2d(
                            reefPoseOffsetX.get(),
                            isCoral.get() ? reefPoseOffsetY.get() : -reefPoseOffsetY.get(),
                            isCoral.get()
                                ? new Rotation2d(Degrees.of(-90))
                                : new Rotation2d(Degrees.of(90))))),
        // robot position supplier
        () -> {
          Optional<Pose2d> txPose =
              drive.getTxTyPose(
                  switch (algaeObjective.get().position().getFace()) {
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
                                  AllianceFlipUtil.apply(
                                          FieldConstants.Reef.centerFaces[
                                              algaeObjective.get().position().getFace()]
                                              .transformBy(
                                                  new Transform2d(
                                                      reefPoseOffsetX.get(),
                                                      isCoral.get()
                                                          ? reefPoseOffsetY.get()
                                                          : -reefPoseOffsetY.get(),
                                                      isCoral.get()
                                                          ? new Rotation2d(Degrees.of(-90))
                                                          : new Rotation2d(Degrees.of(90)))))
                                      .getTranslation())
                          - minDistanceTagPoseBlend.get())
                      / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
                  0.0,
                  1.0);
          return drive.getPose().interpolate(txPose.get(), 1.0 - t);
        });
  }
}
