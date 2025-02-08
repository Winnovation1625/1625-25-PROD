package frc.robot.subsystems.algaemanipulator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class AlgaeManipulator extends SubsystemBase {

  private final AlgaeManipulatorIO io;
  private final AlgaeManipulatorSensorIO sensorsIO;
  private final AlgaeManipulatorSensorIOInputsAutoLogged sensorInputs =
      new AlgaeManipulatorSensorIOInputsAutoLogged();
  private LoggedTunableNumber SensorThreshold =
      new LoggedTunableNumber("Intake/IntakeSensorActivate", 280);
  private LoggedTunableNumber Sensor2Threshold =
      new LoggedTunableNumber("Intake/indexerSensorActivate", 150);
  private boolean algaeInSensor = false;
  private boolean algaeInSensor2 = false;
  
  @RequiredArgsConstructor
  public enum ManipulatorState {
    INTAKING(new LoggedTunableNumber("AlgaeManipulator/AlgaeIntakeVoltage", 12)),
    SHOOTING(new LoggedTunableNumber("AlgaeManipulator/AlgaeShootingVoltage", -12)),
    IDLE(() -> 0);


   private final DoubleSupplier voltageSupplier;

  }

  public enum GamepieceState {
    NONE,
    IN_SHOOTER,
  }

  public AlgaeManipulator(AlgaeManipulatorIO io, AlgaeManipulatorSensorIO sensorsIO) {
    this.io = io;
    this.sensorsIO = sensorsIO;
  }

  
  @AutoLogOutput(key = "AlgaeManipulator/State") 
  @Getter  
  private ManipulatorState state = ManipulatorState.IDLE;

  @AutoLogOutput 
  @Getter 
  private GamepieceState gamepieceState = GamepieceState.NONE;

  private GamepieceState lastGamepieceState = GamepieceState.NONE;
  private Timer gamepieceStateTimer = new Timer();

   @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("IntakeSensors", sensorInputs);

      algaeInSensor =
        sensorInputs.SensorMeasurement < SensorThreshold.get();
    algaeInSensor2 =
        sensorInputs.Sensor2Measurement < Sensor2Threshold.get();

    if (DriverStation.isDisabled()) {
      state = ManipulatorState.IDLE;
    }

    if (algaeInSensor) {
      gamepieceState = GamepieceState.IN_SHOOTER;
    } 
    else {
      gamepieceState = GamepieceState.NONE;
    }
    if (gamepieceState != lastGamepieceState) {
      gamepieceStateTimer.reset();
    }
    lastGamepieceState = gamepieceState;

    // leds
    // switch (gamepieceState) {
    //   case STAGED_IN_SHOOTER -> {
    //     LEDs.getInstance().setNoteInBot(true);
    //     LEDs.getInstance().setNotePickedUp(false);
    //   }
    //   case IN_INDEXER -> {
    //     LEDs.getInstance().setNoteInBot(false);
    //     LEDs.getInstance().setNotePickedUp(true);
    //   }
    //   case HANDOFF -> {
    //     LEDs.getInstance().setNoteInBot(false);
    //     LEDs.getInstance().setNotePickedUp(true);
    //   }
    //   case IN_INTAKE -> {
    //     LEDs.getInstance().setSeenNote(false);
    //     LEDs.getInstance().setNoteInBot(false);
    //     LEDs.getInstance().setNotePickedUp(true);
    //   }
    //   case NONE -> {
    //     LEDs.getInstance().setNoteInBot(false);
    //     LEDs.getInstance().setNotePickedUp(false);
    //     if (!RobotState.getInstance().getNotePose().equals(new Pose3d())) {
    //       LEDs.getInstance().setSeenNote(true);
    //     } else {
    //       LEDs.getInstance().setSeenNote(false);
    //     }
    //   }
    // }

    io.setVoltageOutput(state.voltageSupplier);

  }

   public Command setDesiredStateCommand(ManipulatorState goal) {
    return startEnd(() -> this.state = goal, () -> this.state = ManipulatorState.IDLE)
        .withName("AlgaeManipulator " + goal);
  }



}