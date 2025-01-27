package frc.robot.subsystems.algaemanipulator;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;

public class AlgaeManipulator extends SubsystemBase {


  private final AlgaeManipulatorIO io;
  private final IntakeSensorsIOInputsAutoLogged sensorInputs =
      new IntakeSensorsIOInputsAutoLogged();
  private LoggedTunableNumber intakeSensorThreshold =
      new LoggedTunableNumber("Intake/IntakeSensorActivate", 280);
  private LoggedTunableNumber indexerSensorThreshold =
      new LoggedTunableNumber("Intake/indexerSensorActivate", 150);
  private LoggedTunableNumber stagingSensorThreshold =
      new LoggedTunableNumber("Intake/stagingSensorActivate", 88);
  private boolean noteInInitialIntakeSensor = false;
  private boolean noteInRearIntakeSensor = false;
  private boolean noteInMiddleIntakeSensor = false;
  private boolean noteInInitialIndexerSensor = false;
  private boolean noteInStagingIndexerSensor = false;


  public enum ManipulatorState{

    INTAKING,
    SHOOTING,
    IDLE

  }

  public enum GamepieceState{

    NONE,
    IN_SHOOTER,
    
  }

  public AlgaeManipulator(AlgaeManipulatorIO io){
    io = this.io;
  }

  @Getter
  @AutoLogOutput(key = "AlgaeManipulator/State")
  private ManipulatorState state = ManipulatorState.IDLE;

  @Override
  public void periodic(){

  }
}
