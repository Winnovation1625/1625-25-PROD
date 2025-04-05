package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.leds.Leds;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private Leds leds = Leds.getInstance();
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekI = new LoggedTunableNumber("DriveToPose/DrivekI");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber driveMaxAccelerationSlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxAccelerationSlow");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius");
  private static final LoggedTunableNumber errorDistanceThreshold =
      new LoggedTunableNumber("DriveToPose/ErrorDistanceThreshold");
  private boolean isSlow;

  static {
    drivekP.initDefault(3.5);
    drivekI.initDefault(0.0);
    drivekD.initDefault(0.0);
    thetakP.initDefault(3.5);
    thetakD.initDefault(0.2);
    driveMaxVelocity.initDefault(1.75);
    driveMaxVelocitySlow.initDefault(1.5);
    driveMaxAccelerationSlow.initDefault(1.0);
    driveMaxAcceleration.initDefault(1.5);
    thetaMaxVelocity.initDefault(Units.degreesToRadians(270.0));
    thetaMaxAcceleration.initDefault(4.0);
    driveTolerance.initDefault(Units.inchesToMeters(1.75)); // 2 inch
    thetaTolerance.initDefault(Units.degreesToRadians(2.0));
    ffMinRadius.initDefault(0.00);
    ffMaxRadius.initDefault(0.25);
    errorDistanceThreshold.initDefault(2); // 2 meters
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.LOOP_PERIOD_SECS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.LOOP_PERIOD_SECS);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  @Getter private boolean running = false;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPose(Drive drive, Supplier<Pose2d> target, boolean isSlow) {
    this.drive = drive;
    this.target = target;
    this.isSlow = isSlow;
    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    robot = drive::getPose;
    addRequirements(drive);
  }

  public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot, boolean isSlow) {
    this(drive, target, isSlow);
    this.robot = robot;
  }

  public DriveToPose(
      Drive drive,
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF,
      boolean isSlow) {
    this(drive, target, robot, isSlow);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    driveController.setP(drivekP.get());
    driveController.setI(drivekI.get());
    driveController.setD(drivekD.get());
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(
            isSlow ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
            isSlow ? driveMaxAccelerationSlow.get() : driveMaxAcceleration.get()));
    driveController.setTolerance(driveTolerance.get());
    thetaController.setP(thetakP.get());
    thetaController.setD(thetakD.get());
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
    thetaController.setTolerance(thetaTolerance.get());
    leds.setAutoLiningUp(true);
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveMaxAccelerationSlow.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekI.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setI(drivekI.get());
      driveController.setD(drivekD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              isSlow ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
              isSlow ? driveMaxAccelerationSlow.get() : driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    if (currentDistance > errorDistanceThreshold.get()) {
      leds.setPathingError(true);
    } else {
      leds.setPathingError(false);
    }
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(
            linearFF.get().times(drive.getMaxLinearSpeedMetersPerSec()), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * drive.getMaxAngularSpeedRadPerSec(), thetaS);

    // Command speeds
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    if (atGoal()) {
      leds.setWhenLinedUp(true);
    } else {
      leds.setWhenLinedUp(false);
    }
    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    leds.setAutoLiningUp(false);
    leds.setWhenLinedUp(false);
    leds.setPathingError(false);
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  //   @Override
  //   public boolean isFinished() {
  //     return atGoal();
  //   }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
