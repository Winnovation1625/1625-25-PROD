package frc.robot.subsystem.coralmanipulator;

import org.littletonrobotics.junction.AutoLog;

public interface CoralManipulatorSensorIO {

  @AutoLog
  public static class CoralManipulatorSensorIOInputs {

    public double frontSensorMeasurment = 0.0;
    public double backSensorMeasurment = 0.0;
    public boolean isBackDetected = false;
    public boolean isFrontDetected = false;
  }

  public default void updateInputs(CoralManipulatorSensorIOInputs inputs) {}
}
