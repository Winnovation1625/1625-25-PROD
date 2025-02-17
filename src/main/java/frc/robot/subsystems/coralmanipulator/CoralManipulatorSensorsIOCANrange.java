package frc.robot.subsystems.coralmanipulator;

import com.ctre.phoenix6.hardware.CANrange;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;


import com.ctre.phoenix6.configs.CANrangeConfiguration;

public class CoralManipulatorSensorsIOCANrange implements CoralManipulatorSensorIO {
    
    private final CANrange backSensor;
    private final CANrange frontSensor;
    private final CANrangeConfiguration config = new CANrangeConfiguration();

    public CoralManipulatorSensorsIOCANrange(){

        backSensor = new CANrange(0);
        frontSensor = new CANrange(1);

        try{
            configureSensor(backSensor);
        }
        catch(ConfigurationFailedException e){
            System.out.println("initial intake cannot config " + e.getMessage());
        }
        try{
            configureSensor(frontSensor);
        }
        catch(ConfigurationFailedException e){
            System.out.println("initial intake cannot config " + e.getMessage());
        }
    }

    @Override
    public void updateInputs(CoralManipulatorSensorIOInputs inputs) {
    measurements[0] = backSensor.getMeasurement();
    for (int i = 0; i < 5; i++) {
      if (measurements[i] != null
          && measurements[i].status == ) {
        switch (i) {
          case 0 -> inputs.SensorMeasurement = measurements[i].distance_mm;
        }
      }
    }
  }

     private void configureSensor(CANrange sensor) throws ConfigurationFailedException {
        config.ProximityParams.MinSignalStrengthForValidMeasurement = 2;
  }

}
