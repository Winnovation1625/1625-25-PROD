package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class WristConstants {

  public static final double armTolerance = Units.degreesToRadians(2);
  public static final double minAngle = 0;
  public static final double maxAngle = 75;
  public static final double cruiseVelocity = 0.6;
  public static final double cruiseAcceleration = 2;
  public static final double cruiseJerk = 20;
  public static final double WRIST_TOLERANCE_METERS = Units.inchesToMeters(0.125);
  public static final double WRIST_MASS_KG = 0.125;
  public static final double WRIST_MOI = 0.0040065078;
  public static final double WRIST_GEARING = 81;
  public static final double WRIST_LENGTH = Units.inchesToMeters(6.5);
  public static final double WRIST_MAX_VELOCITY = 0.125;
  public static final double WRIST_MAX_ACCELERATION = 0.125;

  public static Gains gains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(4.0, 0.125, 0.125, 0.0, 0.0, 0.0, 0.0);
        case REAL, REPLAY -> new Gains(600.0, 0.0, 40.0, 3.0, 0.0, 0.0, 4.0);
      };

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
