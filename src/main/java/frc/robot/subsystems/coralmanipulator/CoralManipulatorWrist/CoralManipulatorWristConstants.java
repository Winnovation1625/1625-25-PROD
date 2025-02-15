package frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class CoralManipulatorWristConstants {

    public static final double armTolerance = Units.degreesToRadians(2);
    public static final double minAngle = -33;
    public static final double maxAngle = 64;
    public static final double cruiseVelocity = 0.6; 
    public static final double cruiseAcceleration = 2;
    public static final double cruiseJerk = 20;
    public static final double WRIST_TOLERANCE_METERS = Units.inchesToMeters(0.125);
    public static final double WRIST_PID_P = 0.125;
    public static final double WRIST_PID_I = 0.125;
    public static final double WRIST_PID_D = 0.125;
    public static final double WRIST_MASS_KG = 0.125;
    public static final double WRIST_GEARING = 0.125;


    public static Gains gains =
    switch (Constants.getRobot()) {
      case SIMBOT -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      case TESTBOT -> new Gains(0, 0.0, 0, 0, 21.35, 0.02, 0.12);
      case COMPBOT -> new Gains(600.0, 0.0, 40.0, 3.0, 0.0, 0.0, 4.0);
    };

public record Gains(
    double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
