package frc.robot.subsystem.superstructure.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final double ARM_TOLERANCE = Units.degreesToRadians(0);
  public static final double ARM_KV = 0.125;
  public static final double ARM_KA = 0.125;
  public static final double ARM_GEARING = 25.0;
  public static final double ARM_MOI = 0.0308663843;
  public static final double ARM_MASS_KG = 7.529633;
  public static final double ARM_LENGTH = 1.02108;
  public static final double ARM_PID_P = 4.0;
  public static final double ARM_PID_I = 0.125;
  public static final double ARM_PID_D = 0.125;
  public static final double ARM_MIN_ANGLE_RADS = Units.degreesToRadians(-40);
  public static final double ARM_MAX_ANGLE_RADS = Units.degreesToRadians(200);
  public static final double ARM_MAX_VELOCITY = 5.0;
  public static final double ARM_MAX_ACCELERATION = 2.0;
}
