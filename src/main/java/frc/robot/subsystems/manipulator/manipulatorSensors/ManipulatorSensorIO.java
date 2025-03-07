package frc.robot.subsystems.manipulator.manipulatorSensors;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorSensorIO {

  @AutoLog
  public static class ManipulatorSensorIOInputs {

    public double frontSensorMeasurment = 0.0;
    public double backSensorMeasurment = 0.0;
    public double algaeSensorMeasurment = 0.0;
    public boolean isBackCoralDetected = false;
    public boolean isFrontCoralDetected = false;
    public boolean isAlgaeDetected = false;
  }

  public default void updateInputs(ManipulatorSensorIOInputs inputs) {}
}
