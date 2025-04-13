package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
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

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(Drive.ROBOT_MASS_KG))
          .withBumperSize(Inches.of(35.437500), Inches.of(31.937500))
          .withCustomModuleTranslations(Drive.getModuleTranslations())
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
                  Drive.WHEEL_COF));
}
