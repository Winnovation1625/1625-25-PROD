package frc.robot.subsystem.superstructure.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmConstants {
  public static final double ARM_TOLERANCE = Units.degreesToRadians(0.5);
  public static final double ARM_KV = 0.125;
  public static final double ARM_KA = 0.125;
  public static final double ARM_GEARING = 25.0;
  public static final double ARM_MOI = 0.0308663843;
  public static final double ARM_MASS_KG = 5.06571959;
  public static final double ARM_LENGTH = 1.02108;
  public static final double ARM_MIN_ANGLE_RADS = Units.degreesToRadians(-40);
  public static final double ARM_MAX_ANGLE_RADS = Units.degreesToRadians(200);
  public static final double ARM_MAX_VELOCITY = 25.0;
  public static final double ARM_MAX_ACCELERATION = 50.0;

  public static Gains gains =
      switch (Constants.CURRENT_MODE) {
        case SIM -> new Gains(4.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL, REPLAY -> new Gains(600.0, 0.0, 40.0, 3.0, 0.0, 0.0, 4.0);
      };

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
