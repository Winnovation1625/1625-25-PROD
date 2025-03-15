package frc.robot.subsystem.manipulator.manipulatorSensors;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystem.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ManipulatorSensorsIOCANrange implements ManipulatorSensorIO {

  private final CANrange backSensor;
  private final CANrange frontSensor;
  private final CANrange algaeSensor;
  private final StatusSignal<Boolean> isFrontDetected;
  private final StatusSignal<Boolean> isBackDetected;
  private final StatusSignal<Boolean> isAlgaeDetected;
  private final StatusSignal<Distance> frontSensorValue;
  private final StatusSignal<Distance> backSensorValue;
  private final StatusSignal<Distance> algaeSensorValue;

  public ManipulatorSensorsIOCANrange() {

    backSensor =
        new CANrange(Constants.MANIPULATOR_CAN_IDS.coralBackCANRange(), Constants.CANIVORE_NAME);
    frontSensor =
        new CANrange(Constants.MANIPULATOR_CAN_IDS.coralFrontCANRange(), Constants.CANIVORE_NAME);
    algaeSensor =
        new CANrange(Constants.MANIPULATOR_CAN_IDS.algaeCANRange(), Constants.CANIVORE_NAME);

    backSensor.getConfigurator().apply(CORAL_SENSOR_CONFIG);
    frontSensor.getConfigurator().apply(CORAL_SENSOR_CONFIG);
    algaeSensor.getConfigurator().apply(ALGAE_SENSOR_CONFIG);

    isFrontDetected = frontSensor.getIsDetected();
    isBackDetected = backSensor.getIsDetected();
    isAlgaeDetected = algaeSensor.getIsDetected();
    frontSensorValue = frontSensor.getDistance();
    backSensorValue = backSensor.getDistance();
    algaeSensorValue = algaeSensor.getDistance();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        isBackDetected,
        isFrontDetected,
        isAlgaeDetected,
        frontSensorValue,
        backSensorValue,
        algaeSensorValue);
  }

  @Override
  public void updateInputs(ManipulatorSensorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        isFrontDetected,
        isBackDetected,
        isAlgaeDetected,
        frontSensorValue,
        backSensorValue,
        algaeSensorValue);
    inputs.frontSensorMeasurment = frontSensorValue.getValue().in(Inches);
    inputs.backSensorMeasurment = backSensorValue.getValue().in(Inches);
    inputs.algaeSensorMeasurment = algaeSensorValue.getValue().in(Inches);
    inputs.isBackCoralDetected = isBackDetected.getValue();
    inputs.isFrontCoralDetected = isFrontDetected.getValue();
    inputs.isAlgaeDetected = isAlgaeDetected.getValue();
  }
}
