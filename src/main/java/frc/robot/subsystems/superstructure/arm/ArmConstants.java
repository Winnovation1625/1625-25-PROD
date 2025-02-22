package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final double ARM_TOLERANCE = Units.degreesToRadians(2);
  public static final double ARM_KV = 0.0;
  public static final double ARM_KA = 0.0;
  public static final double ARM_GEARING = 1.0;
  public static final double ARM_MOI = 1.0;
  public static final double ARM_MASS_KG = 0.125;
  public static final double ARM_LENGTH = 2.0;
  public static final double ARM_PID_P = 0.125;
  public static final double ARM_PID_I = 0.125;
  public static final double ARM_PID_D = 0.125;
  public static final double ARM_MIN_ANGLE_RADS = Units.degreesToRadians(-33);
  public static final double ARM_MAX_ANGLE_RADS = Units.degreesToRadians(64);
  public static final double ARM_MAX_VELOCITY = 0.125;
  public static final double ARM_MAX_ACCELERATION = 0.125;
}
