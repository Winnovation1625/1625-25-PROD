package frc.robot.subsystems.coralmanipulator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralManipulator extends SubsystemBase {

  private final CoralManipulatoIO io;
  private final CoralManipulatorSensorIO sensorIO;
  private final CoralManipulatorIOInputsAutoLogged inputs =
      new CoralManipulatorIOInputsAutoLogged();
  private final CoralManipulatorSensorIOInputsAutoLogged sensorInputs =
      new CoralManipulatorSensorIOInputsAutoLogged();
  private boolean algaeInSensor = false;
  private boolean onlyBackSensor = false;

  @RequiredArgsConstructor
  public enum RollersState {
    INTAKING(new LoggedTunableNumber("CoralManipulator/Intaking", 10)),
    SHOOTING(new LoggedTunableNumber("CoralManipulator/Shooting", 10)),
    EJECTING(new LoggedTunableNumber("CoralManipulator/Ejecting", 20)),
    IDLE(() -> 0);

    private final DoubleSupplier voltageSupplier;
  }

  @RequiredArgsConstructor
  public enum GamepieceState {
    NONE,
    IN_INTAKE;
  }

  public CoralManipulator(CoralManipulatoIO io, CoralManipulatorSensorIO sensorIO) {
    this.io = io;
    this.sensorIO = sensorIO;
  }

  @AutoLogOutput(key = "CoralManipulator/GamepieceState")
  @Getter
  private GamepieceState gamepieceState = GamepieceState.NONE;

  @AutoLogOutput(key = "CoralManipulator/RollersState")
  @Getter
  private RollersState rollersState = RollersState.IDLE;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    sensorIO.updateInputs(sensorInputs);

    algaeInSensor = sensorInputs.isBackDetected && sensorInputs.isFrontDetected;

    if (sensorInputs.isBackDetected == true && sensorInputs.isFrontDetected == false) {
      onlyBackSensor = true;
    }

    if (algaeInSensor == true) {
      gamepieceState = GamepieceState.IN_INTAKE;
      rollersState = RollersState.IDLE;
    } else if (onlyBackSensor == true) {
      rollersState = RollersState.INTAKING;
    }

    Logger.processInputs("CoralManipulator", inputs);

    if (DriverStation.isDisabled()) {
      rollersState = RollersState.IDLE;
    }

    io.setVoltage(rollersState.voltageSupplier);
  }

  public Command setDesiredStateCommand(RollersState goal) {
    return startEnd(() -> this.rollersState = goal, () -> this.rollersState = RollersState.IDLE)
        .withName("CoralManipulator " + goal);
  }
}
