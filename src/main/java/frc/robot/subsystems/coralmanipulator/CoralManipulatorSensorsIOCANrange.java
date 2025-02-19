package frc.robot.subsystems.coralmanipulator;

import static frc.robot.subsystems.coralmanipulator.CoralManipulatorConstants.constCoralOuttake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorConstants.constCoralOuttake;

public class CoralManipulatorSensorsIOCANrange implements CoralManipulatorSensorIO {

  private final CANrange backSensor;
  private final CANrange frontSensor;
  private final StatusSignal<Boolean> isFrontDetected;
  private final StatusSignal<Boolean> isBackDetected;

  public CoralManipulatorSensorsIOCANrange() {

    backSensor = new CANrange(0);
    frontSensor = new CANrange(1);

    backSensor.getConfigurator().apply(constCoralOuttake.CORAL_SENSOR_CONFIG);
    frontSensor.getConfigurator().apply(constCoralOuttake.CORAL_SENSOR_CONFIG);

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
