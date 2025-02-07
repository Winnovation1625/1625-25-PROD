package frc.robot.subsystems.algaemanipulator;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class AlgaeManipulatorIOSensor implements AlgaeManipulatorIO {
    
    private final LaserCan sensor;

    private LaserCan.Measurement[] measurements = new LaserCan.Measurement[5];

    public AlgaeManipulatorIOSensor(){

        sensor = new LaserCan(0); // constants ID is needed

        try {
         configureLaserCans(sensor);
        } catch (ConfigurationFailedException e) {
        System.out.println("initial intake cannot config " + e.getMessage());
        }


    }

    private void configureLaserCans(LaserCan laserCan) throws ConfigurationFailedException {
    laserCan.setRangingMode(RangingMode.SHORT);
    laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
  }

}
