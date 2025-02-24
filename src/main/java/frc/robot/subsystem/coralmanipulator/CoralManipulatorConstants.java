package frc.robot.subsystem.coralmanipulator;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class CoralManipulatorConstants {

  public static final Distance REQUIRED_CORAL_DISTANCE = Units.Meters.of(0.1);
  public static final double ROLLERS_REDUCTION = 12.0;
  public static CANrangeConfiguration CORAL_SENSOR_CONFIG = new CANrangeConfiguration();

  static {
    CORAL_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    CORAL_SENSOR_CONFIG.ProximityParams.ProximityThreshold =
        REQUIRED_CORAL_DISTANCE.in(Units.Meters);
  }

  public static class constCoralOuttake {
    public static final double CORAL_OUTTAKE_SPEED = 0.7;
    public static final double CORAL_L1_OUTTAKE_SPEED = 0.2;

    public static final double CORAL_L4_OUTTAKE_SPEED = 0.4;

    public static final double CORAL_INTAKE_SPEED = 0.8;
    public static final double CORAL_INDEXING_SPEED = 0.15;

    public static final Distance REQUIRED_CORAL_DISTANCE = Units.Meters.of(0.1);
    public static final Distance INDEXED_CORAL_DISTANCE = Units.Meters.of(0.13);

    public static final Time CORAL_SCORE_TIME = Units.Seconds.of(0.3);

    public static TalonFXConfiguration CORAL_OUTTAKE_CONFIG = new TalonFXConfiguration();

    static {
      CORAL_OUTTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      CORAL_OUTTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
  }

  public static final Gains gains =
      switch (Constants.CURRENT_MODE) {
        case REAL, REPLAY -> new Gains(0.3, 0, 0.0016, 0.17523, 0.084, 0);
        case SIM -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
