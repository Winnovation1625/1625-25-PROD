package frc.robot.subsystem.superstructure.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class ArmConstants {
  public static final double ARM_TOLERANCE = Units.degreesToRadians(0.5);
  public static final double ARM_KV = 0.125;
  public static final double ARM_KA = 0.125;
  public static final double ARM_GEARING = 75.0;
  public static final double ARM_MOI = 0.0308663843;
  public static final double ARM_MASS_KG = 5.06571959;
  public static final double ARM_LENGTH = 1.02108;
  public static final Angle ARM_ENCODER_OFFSET = Radian.of(-1.8438449070385408);
  public static final Angle ARM_MIN_ANGLE_RADS = Rotations.of(-0.3);
  public static final Angle ARM_MAX_ANGLE_RADS = Rotations.of(0.4);
  public static final double ARM_MAX_VELOCITY = 1.0;
  public static final double ARM_MAX_ACCELERATION = 4.0;
  public static final boolean ARM_INVERTED = true;

  public static Gains gains =
      switch (Constants.CURRENT_MODE) {
        case SIM -> new Gains(4.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 200.0, 400.0);
        case REAL, REPLAY -> new Gains(220, 100.0, 140.0, 6.0, 2.0, 1.0, 3.5, 1, 4);
      };

  public record Gains(
      double kP,
      double kI,
      double kD,
      double ffkS,
      double ffkV,
      double ffkA,
      double ffkG,
      double cruiseVelocity,
      double cruiseAcceleration) {}
}
