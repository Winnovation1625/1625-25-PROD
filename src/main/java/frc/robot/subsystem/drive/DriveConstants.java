package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.TunerConstants;

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
  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
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
    
}
