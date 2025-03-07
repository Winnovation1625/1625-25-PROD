package frc.robot.subsystem.manipulator.manipulatorSensors;

import frc.robot.util.LoggedTunableNumber;

public class ManipulatorSensorIOSim implements ManipulatorSensorIO {

  private LoggedTunableNumber frontSensor =
      new LoggedTunableNumber("ManipulatorSensorIOSim/Frontsensor", 290);
  private LoggedTunableNumber backSensor =
      new LoggedTunableNumber("ManipulatorSensorIOSim/Backsensor", 290);
  private LoggedTunableNumber algaeSensor =
      new LoggedTunableNumber("ManipulatorSensorIOSim/Algaesensor", 290);

  @Override
  public void updateInputs(ManipulatorSensorIOInputs inputs) {
    inputs.frontSensorMeasurment = frontSensor.get();
    inputs.backSensorMeasurment = backSensor.get();
    inputs.algaeSensorMeasurment = algaeSensor.get();
  }
}
