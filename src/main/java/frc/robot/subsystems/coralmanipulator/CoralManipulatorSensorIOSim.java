package frc.robot.subsystems.coralmanipulator;

import frc.robot.util.LoggedTunableNumber;

public class CoralManipulatorSensorIOSim implements CoralManipulatorSensorIO {

  private LoggedTunableNumber Frontsensor =
      new LoggedTunableNumber("CoralManipulatorSensorIOSim/Frontsensor", 40.0);
  private LoggedTunableNumber Backsensor =
      new LoggedTunableNumber("CoralManipulatorSensorIOSim/Backsensor", 40.0);

  private double sensorThreshold = 20.0;

  @Override
  public void updateInputs(CoralManipulatorSensorIOInputs inputs) {
    inputs.frontSensorMeasurment = Frontsensor.getAsDouble();
    inputs.backSensorMeasurment = Backsensor.getAsDouble();
    inputs.isFrontDetected = sensorThreshold > Frontsensor.getAsDouble();
    inputs.isBackDetected = sensorThreshold > Backsensor.getAsDouble();
  }
}
