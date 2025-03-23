package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants.HumanStation;
import frc.robot.subsystem.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

public class DriveToHumanStation extends DriveToPose {
  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber(
          "DriveToHumanStation/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber(
          "DriveToHumanStation/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber humanPlayerOffsetX =
      new LoggedTunableNumber("DriveToHumanStation/HumanPlayerXOffset", Units.inchesToMeters(16.5));
  private static final LoggedTunableNumber humanPlayerOffsetY =
      new LoggedTunableNumber("DriveToHumanStation/HumanPlayerYOffset", Units.inchesToMeters(0.0));

  static {
    minDistanceTagPoseBlend.initDefault(Units.inchesToMeters(24.0));
    maxDistanceTagPoseBlend.initDefault(Units.inchesToMeters(36.0));
    humanPlayerOffsetX.initDefault(Units.inchesToMeters(16.5));
    humanPlayerOffsetY.initDefault(Units.inchesToMeters(0.0));
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
                            humanPlayerOffsetX.get(),
                            humanPlayerOffsetY.get(),
                            new Rotation2d(Degrees.of(0))))),
        // robot position supplier
        drive::getPose);
  }
}
