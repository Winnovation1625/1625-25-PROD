package frc.robot.subsystems.coralmanipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollers;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollersIO;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist.CoralManipulatorWrist;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist.CoralManipulatorWristIO;

public class CoralManipulator extends SubsystemBase{

    private final CoralManipulatorWrist wrist;
    private final CoralManipulatorRollers rollers;
    private final CoralManipulatorSensorIO sensorsIO;
    private final CoralManipulatorSensorIOInputsAutoLogged sensorInputs = new CoralManipulatorSensorIOInputsAutoLogged();

    public CoralManipulator(CoralManipulatorWristIO wristIO, CoralManipulatorRollersIO rollersIO, CoralManipulatorSensorIO sensorsIO){
        wrist = new CoralManipulatorWrist(wristIO);
        rollers = new CoralManipulatorRollers(rollersIO);
        this.sensorsIO = sensorsIO;
    }

    public enum CoralManipulatorStates{

        INTAKING,
        INDEXING,
        LEVEL_ONE,
        LEVEL_MID,
        LEVEL_FOUR,
        START_CONFIG,
        STOW,
        STOP,


    }

    @Override
    public void periodic(){

        wrist.periodic();
        rollers.periodic();
        sensorsIO.updateInputs(sensorInputs);



    }


}
