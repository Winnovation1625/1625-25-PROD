package frc.robot.subsystems.algaemanipulator;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeManipulatorSensorIO {

  @AutoLog
  class AlgaeManipulatorSensorIOInputs {

    public double SensorMeasurement = 1000;
  }

  public default void updateInputs(AlgaeManipulatorSensorIOInputs inputs) {}
}
