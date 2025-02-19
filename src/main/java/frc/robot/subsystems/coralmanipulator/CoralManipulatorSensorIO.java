package frc.robot.subsystems.coralmanipulator;

import org.littletonrobotics.junction.AutoLog;

public interface CoralManipulatorSensorIO {

  @AutoLog
  public static class CoralManipulatorSensorIOInputs {

    public double frontSensorMeasurment = 0.0;
    public double backSensorMeasurment = 0.0;
    public Boolean isBackDetected;
    public Boolean isFrontDetected; 

  }

  public default void updateInputs(CoralManipulatorSensorIOInputs inputs) {}
  
}
