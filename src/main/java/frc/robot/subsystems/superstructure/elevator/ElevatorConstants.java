package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final double ELEVATOR_TOLERANCE_METERS = 0.125;
  public static final double ELEVATOR_KV = 1.91;
  public static final double ELEVATOR_KA = 0.04;
  public static final double ELEVATOR_KG = 0.39;
  public static final double ELEVATOR_GEARING = 5.0;
  public static final double ELEVATOR_MOI = 0.125;
  public static final double ELEVATOR_DRUM_RADIUS = 0.8125;
  public static final double ELEVATOR_DRUM_MOI = 0.125;
  public static final double ELEVATOR_MIN_HEIGHT_METERS = Units.inchesToMeters(11.75);
  public static final double ELEVATOR_MAX_HEIGHT_METERS = Units.inchesToMeters(84.75);
  public static final double ELEVATOR_MASS_KG = 14.8882624;
  public static final double ELEVATOR_PID_P = 32.0;
  public static final double ELEVATOR_PID_I = 0.125;
  public static final double ELEVATOR_PID_D = 0.125;
  public static final double ELEVATOR_MAX_VELOCITY = 8.94;
  public static final double ELEVATOR_MAX_ACCELERATION = 4.47;
}
