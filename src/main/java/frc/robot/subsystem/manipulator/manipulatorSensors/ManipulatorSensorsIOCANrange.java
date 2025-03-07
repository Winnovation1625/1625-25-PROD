package frc.robot.subsystem.manipulator.manipulatorSensors;

import static frc.robot.subsystem.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.Units;

public class ManipulatorSensorsIOCANrange implements ManipulatorSensorIO {

  private final CANrange backSensor;
  private final CANrange frontSensor;
  private final CANrange algaeSensor;
  private final StatusSignal<Boolean> isFrontDetected;
  private final StatusSignal<Boolean> isBackDetected;
  private final StatusSignal<Boolean> isAlgaeDetected;
  private final CANrangeConfiguration CORAL_SENSOR_CONFIG;
  private final CANrangeConfiguration ALGAE_SENSOR_CONFIG;

  public ManipulatorSensorsIOCANrange() {

    backSensor = new CANrange(0);
    frontSensor = new CANrange(1);
    algaeSensor = new CANrange(2);

    CORAL_SENSOR_CONFIG = new CANrangeConfiguration();
    ALGAE_SENSOR_CONFIG = new CANrangeConfiguration();

    CORAL_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    CORAL_SENSOR_CONFIG.ProximityParams.ProximityThreshold =
        REQUIRED_CORAL_DISTANCE.in(Units.Meters);

    backSensor.getConfigurator().apply(CORAL_SENSOR_CONFIG);
    frontSensor.getConfigurator().apply(CORAL_SENSOR_CONFIG);
    algaeSensor.getConfigurator().apply(ALGAE_SENSOR_CONFIG);

    isFrontDetected = frontSensor.getIsDetected();
    isBackDetected = backSensor.getIsDetected();
    isAlgaeDetected = algaeSensor.getIsDetected();
  }

  @Override
  public void updateInputs(ManipulatorSensorIOInputs inputs) {

    inputs.frontSensorMeasurment = frontSensor.getDistance().getValueAsDouble();
    inputs.backSensorMeasurment = backSensor.getDistance().getValueAsDouble();
    inputs.algaeSensorMeasurment = algaeSensor.getDistance().getValueAsDouble();
    inputs.isBackCoralDetected = backSensor.getIsDetected().getValue();
    inputs.isFrontCoralDetected = frontSensor.getIsDetected().getValue();
    inputs.isAlgaeDetected = algaeSensor.getIsDetected().getValue();
  }
}
