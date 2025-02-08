package frc.robot.subsystems.algaemanipulator;

import frc.robot.util.LoggedTunableNumber;

public class AlgaeManipulatorSensorIOSim implements AlgaeManipulatorSensorIO {

  private LoggedTunableNumber sensor =
      new LoggedTunableNumber("AlgaeManipulatorSensorIOSim/sensor", 290);

  @Override
  public void updateInputs(AlgaeManipulatorSensorIOInputs inputs) {
    inputs.SensorMeasurement = sensor.get();
  }
}
