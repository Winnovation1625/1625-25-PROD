package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.TunerConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants extends TunerConstants {

  static final double ODOMETRY_FREQUENCY =
      new CANBus(DriveConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
              Math.hypot(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
              Math.hypot(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)));

  // PathPlanner config constants
  public static final double ROBOT_MASS_KG = 61.507125;
  public static final double ROBOT_MOI = 6.5666772511;
  public static final double WHEEL_COF = 1.2;

  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          DriveConstants.ROBOT_MASS_KG,
          DriveConstants.ROBOT_MOI,
          new ModuleConfig(
              DriveConstants.FrontLeft.WheelRadius,
              DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
              DriveConstants.WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(DriveConstants.FrontLeft.DriveMotorGearRatio),
              DriveConstants.FrontLeft.SlipCurrent,
              1),
          DriveConstants.getModuleTranslations());

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
      new Translation2d(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY),
      new Translation2d(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
      new Translation2d(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)
    };
  }

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
          .withBumperSize(Inches.of(35.437500), Inches.of(31.937500))
          .withCustomModuleTranslations(getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getFalcon500(1),
                  FrontLeft.DriveMotorGearRatio,
                  FrontLeft.SteerMotorGearRatio,
                  Volts.of(FrontLeft.DriveFrictionVoltage),
                  Volts.of(FrontLeft.SteerFrictionVoltage),
                  Meters.of(FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(FrontLeft.SteerInertia),
                  WHEEL_COF));
}
