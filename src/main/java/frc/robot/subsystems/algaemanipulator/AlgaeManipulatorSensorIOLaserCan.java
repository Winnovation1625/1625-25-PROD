package frc.robot.subsystems.algaemanipulator;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class AlgaeManipulatorSensorIOLaserCan implements AlgaeManipulatorSensorIO {

  private final LaserCan Sensor; // Will ask about names later.
  private final LaserCan Sensor2;

  private LaserCan.Measurement[] measurements = new LaserCan.Measurement[2];

  public AlgaeManipulatorSensorIOLaserCan() {

    Sensor = new LaserCan(0); // constants ID is needed
    Sensor2 = new LaserCan(1);

    try {
      configureLaserCans(Sensor);
    } catch (ConfigurationFailedException e) {
      System.out.println("initial intake cannot config " + e.getMessage());
    }
    try {
      configureLaserCans(Sensor2);
    } catch (ConfigurationFailedException e) {
      System.out.println("initial intake cannot config " + e.getMessage());
    }
  }

  @Override
  public void updateInputs(AlgaeManipulatorSensorIOInputs inputs) {
    measurements[0] = Sensor.getMeasurement();
    measurements[1] = Sensor2.getMeasurement();
    for (int i = 0; i < 5; i++) {
      if (measurements[i] != null
          && measurements[i].status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        switch (i) {
          case 0 -> inputs.SensorMeasurement = measurements[i].distance_mm;
          case 1 -> inputs.Sensor2Measurement = measurements[i].distance_mm;
        }
      }
    }
  }

  private void configureLaserCans(LaserCan laserCan) throws ConfigurationFailedException {
    laserCan.setRangingMode(RangingMode.SHORT);
    laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
  }
}
