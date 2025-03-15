package frc.robot.subsystem.manipulator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;

public class ManipulatorConstants {

  public static final Distance REQUIRED_CORAL_DISTANCE = Inches.of(4);
  public static final Distance REQUIRED_ALGAE_DISTANCE = Inches.of(4);
  public static final double ROLLERS_REDUCTION = 3.0;
  public static CANrangeConfiguration CORAL_SENSOR_CONFIG = new CANrangeConfiguration();
  public static CANrangeConfiguration ALGAE_SENSOR_CONFIG = new CANrangeConfiguration();

  static {
    CORAL_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    CORAL_SENSOR_CONFIG.ToFParams.UpdateFrequency = 50;
    CORAL_SENSOR_CONFIG.FovParams.FOVCenterX = 0;
    CORAL_SENSOR_CONFIG.FovParams.FOVCenterY = 0;
    CORAL_SENSOR_CONFIG.FovParams.FOVRangeX = 10;
    CORAL_SENSOR_CONFIG.FovParams.FOVRangeY = 10;
    CORAL_SENSOR_CONFIG.ProximityParams.ProximityThreshold = REQUIRED_CORAL_DISTANCE.in(Meters);
    CORAL_SENSOR_CONFIG.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    CORAL_SENSOR_CONFIG.ProximityParams.ProximityHysteresis = 0.005;

    ALGAE_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    ALGAE_SENSOR_CONFIG.ToFParams.UpdateFrequency = 50;
    ALGAE_SENSOR_CONFIG.FovParams.FOVCenterX = 0;
    ALGAE_SENSOR_CONFIG.FovParams.FOVCenterY = 0;
    ALGAE_SENSOR_CONFIG.FovParams.FOVRangeX = 20;
    ALGAE_SENSOR_CONFIG.FovParams.FOVRangeY = 20;
    ALGAE_SENSOR_CONFIG.ProximityParams.ProximityThreshold = REQUIRED_CORAL_DISTANCE.in(Meters);
    ALGAE_SENSOR_CONFIG.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    ALGAE_SENSOR_CONFIG.ProximityParams.ProximityHysteresis = 0.01;
  }

  // public static class constCoralOuttake {
  //   public static final double CORAL_OUTTAKE_SPEED = 0.7;
  //   public static final double CORAL_L1_OUTTAKE_SPEED = 0.2;

  //   public static final double CORAL_L4_OUTTAKE_SPEED = 0.4;

  //   public static final double CORAL_INTAKE_SPEED = 0.8;
  //   public static final double CORAL_INDEXING_SPEED = 0.15;

  //   public static final Distance REQUIRED_CORAL_DISTANCE = Units.Meters.of(0.01);
  //   public static final Distance INDEXED_CORAL_DISTANCE = Units.Meters.of(0.13);

  //   public static final Time CORAL_SCORE_TIME = Units.Seconds.of(0.3);

  //   public static TalonFXConfiguration CORAL_OUTTAKE_CONFIG = new TalonFXConfiguration();

  //   static {
  //     CORAL_OUTTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  //     CORAL_OUTTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  //   }
  // }

  // public static final Gains gains =
  //     switch (Constants.CURRENT_MODE) {
  //       case REAL, REPLAY -> new Gains(0.3, 0, 0.0016, 0.17523, 0.084, 0);
  //       case SIM -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
  //     };

  // public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
