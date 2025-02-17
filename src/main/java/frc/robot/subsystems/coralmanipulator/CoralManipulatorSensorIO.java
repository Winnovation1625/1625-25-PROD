package frc.robot.subsystems.coralmanipulator;

import org.littletonrobotics.junction.AutoLog;

public interface CoralManipulatorSensorIO {

    @AutoLog
    public class CoralManipulatorSensorIOInputs{

        public double sensorMeasurment = 200;

    }

    public default void updateInputs(CoralManipulatorSensorIOInputs inputs) {}
    
}
