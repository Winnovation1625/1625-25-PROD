package frc.robot.subsystem.manipulator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.leds.Leds;
import frc.robot.subsystem.manipulator.manipulatorSensors.ManipulatorSensorIO;
import frc.robot.subsystem.manipulator.manipulatorSensors.ManipulatorSensorIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {

  private final ManipulatorIO io;
  private final ManipulatorSensorIO sensorsIO;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private final ManipulatorSensorIOInputsAutoLogged sensorInputs =
      new ManipulatorSensorIOInputsAutoLogged();

  @RequiredArgsConstructor
  public enum ManipulatorState {
    INTAKING_CORAL(new LoggedTunableNumber("Manipulator/CoralIntakeVoltage", -3)),
    INTAKING_ALGAE(new LoggedTunableNumber("Manipulator/AlgaeIntakeVoltage", 4)),
    STAGING_CORAL(new LoggedTunableNumber("Manipulator/CoralStagingVoltage", -1.5)),
    SHOOTING_CORAL(new LoggedTunableNumber("Manipulator/CoralShootingVoltage", -6)),
    SHOOTING_ALGAE(new LoggedTunableNumber("Manipulator/AlgaeShootingVoltage", -12)),
    IDLE(() -> 0);

    private final DoubleSupplier voltageSupplier;
  }

  @RequiredArgsConstructor
  public enum GamepieceState {
    ALGAE_IN_CLAW,
    CORAL_STAGING,
    CORAL_IN_MANIPULATOR,
    NONE;
  }

  public Manipulator(ManipulatorIO io, ManipulatorSensorIO sensorsIO) {
    this.io = io;
    this.sensorsIO = sensorsIO;
  }

  @AutoLogOutput(key = "Manipulator/GamepieceState")
  @Getter
  private GamepieceState gamepieceState = GamepieceState.NONE;

  @AutoLogOutput(key = "Manipulator/ManipulatorState")
  @Getter
  private ManipulatorState manipulatorState = ManipulatorState.IDLE;

  private ManipulatorState prevManipulatorState = ManipulatorState.IDLE;

  public boolean hasAlgae() {
    return gamepieceState == GamepieceState.ALGAE_IN_CLAW;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("Manipulator/Sensors", sensorInputs);

    if (sensorInputs.isFrontCoralDetected == false && sensorInputs.isBackCoralDetected == true) {
      manipulatorState = ManipulatorState.STAGING_CORAL;
      gamepieceState = GamepieceState.CORAL_STAGING;
    }

    if (sensorInputs.isFrontCoralDetected == true
        && (sensorInputs.isBackCoralDetected == true
            || manipulatorState == ManipulatorState.SHOOTING_CORAL)) {
      gamepieceState = GamepieceState.CORAL_IN_MANIPULATOR;
      if (prevManipulatorState == ManipulatorState.STAGING_CORAL) {
        manipulatorState = ManipulatorState.IDLE;
      }
      Leds.getInstance().setCoralInBot(true);
      Leds.getInstance().setAlgaeInBot(false);
    }

    if (!sensorInputs.isFrontCoralDetected && !sensorInputs.isBackCoralDetected) {
      gamepieceState = GamepieceState.NONE;
    }

    if (sensorInputs.isAlgaeDetected == true) {
      gamepieceState = GamepieceState.ALGAE_IN_CLAW;
      if (prevManipulatorState == ManipulatorState.INTAKING_ALGAE) {
        manipulatorState = ManipulatorState.IDLE;
      }
      Leds.getInstance().setAlgaeInBot(true);
      Leds.getInstance().setCoralInBot(false);
    }

    if (DriverStation.isDisabled()) {
      manipulatorState = ManipulatorState.IDLE;
    }
    prevManipulatorState = manipulatorState;
    io.setVoltage(manipulatorState.voltageSupplier);
  }

  public Command setManipulatorState(ManipulatorState state) {
    return Commands.runOnce(() -> runManipulator(state));
  }

  public Command runManipulator(ManipulatorState state) {
    manipulatorState = state;
    return Commands.runOnce(() -> io.setVoltage(state.voltageSupplier));
  }
}
