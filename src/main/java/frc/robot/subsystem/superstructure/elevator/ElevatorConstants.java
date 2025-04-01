package frc.robot.subsystem.superstructure.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {
  public static final double ELEVATOR_TOLERANCE_METERS = 0.02;
  public static final double ELEVATOR_KV = 1.91;
  public static final double ELEVATOR_KA = 0.04;
  public static final double ELEVATOR_KG = 0.39;
  public static final double ELEVATOR_GEARING = 5.0;
  public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(1);
  public static final double ELEVATOR_OFFSET_ROTATIONS = 2.6;
  public static final double ELEVATOR_MIN_HEIGHT_METERS = 0;
  public static final double ELEVATOR_MAX_HEIGHT_METERS = 1.76;
  public static final double ELEVATOR_MASS_KG = 12.42434899;
  public static final double ELEVATOR_MAX_VELOCITY = 28.812;
  public static final double ELEVATOR_MAX_ACCELERATION = 63.769;
  public static final double ELEVATOR_FIRST_STAGE_TRAVEL_DISTANCE = Units.inchesToMeters(26.0);
  public static final double ELEVATOR_SECOND_STAGE_TRAVEL_DISTANCE = Units.inchesToMeters(25.0);
  public static final double ELEVATOR_CARRIAGE_TRAVEL_DISTANCE = Units.inchesToMeters(22.0);

  // public static Gains followerGains =
  //     switch (Constants.CURRENT_MODE) {
  //       case SIM -> new Gains(0.5, 0.0, 0.0, 40.0, 3.0, 0.0, 4.0, 0.1, 0.5, 0.0);
  //       case REAL, REPLAY -> new Gains(0.5, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.0);
  //     };
  public static Gains positionGains =
      switch (Constants.CURRENT_MODE) {
        case SIM -> new Gains(1, 0.0, 0.0, 40.0, 3.0, 0.0, 4.0, 0.1, 0.5, 0.0);
        case REAL, REPLAY -> new Gains(180, 25.0, 60.0, 10.0, 9.0, 2.2, 15.0, 18.0, 35.0, 0.0);
      };

  public record Gains(
      double kP,
      double kI,
      double kD,
      double ffkS,
      double ffkV,
      double ffkA,
      double ffkG,
      double cruiseV,
      double cruiseA,
      double cruiseJ) {}
}
