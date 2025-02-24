package frc.robot.subsystem.coralmanipulator;

import static frc.robot.subsystem.coralmanipulator.CoralManipulatorConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.Units;

public class CoralManipulatorSensorsIOCANrange implements CoralManipulatorSensorIO {

  private final CANrange backSensor;
  private final CANrange frontSensor;
  private final StatusSignal<Boolean> isFrontDetected;
  private final StatusSignal<Boolean> isBackDetected;
  private final CANrangeConfiguration CORAL_SENSOR_CONFIG;

  public CoralManipulatorSensorsIOCANrange() {

    backSensor = new CANrange(0);
    frontSensor = new CANrange(1);

    CORAL_SENSOR_CONFIG = new CANrangeConfiguration();

    CORAL_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    CORAL_SENSOR_CONFIG.ProximityParams.ProximityThreshold =
        REQUIRED_CORAL_DISTANCE.in(Units.Meters);

    backSensor.getConfigurator().apply(CORAL_SENSOR_CONFIG);
    frontSensor.getConfigurator().apply(CORAL_SENSOR_CONFIG);

    isFrontDetected = frontSensor.getIsDetected();
    isBackDetected = backSensor.getIsDetected();
  }

  @Override
  public void updateInputs(CoralManipulatorSensorIOInputs inputs) {

    inputs.frontSensorMeasurment = frontSensor.getDistance().getValueAsDouble();
    inputs.backSensorMeasurment = backSensor.getDistance().getValueAsDouble();
    inputs.isBackDetected = backSensor.getIsDetected().getValue();
    inputs.isFrontDetected = frontSensor.getIsDetected().getValue();
  }
}
