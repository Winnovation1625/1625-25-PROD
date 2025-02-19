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
  private final CoralManipulatorIOInputsAutoLogged inputs =
      new CoralManipulatorIOInputsAutoLogged();

  @RequiredArgsConstructor
  public enum RollersState {
    INTAKING(new LoggedTunableNumber("CoralManipulator/Intaking", 10)),
    SHOOTING(new LoggedTunableNumber("CoralManipulator/Shooting", 10)),
    EJECTING(new LoggedTunableNumber("CoralManipulator/Ejecting", 20)),
    IDLE(() -> 0);

    private final DoubleSupplier voltageSupplier;
  }

  public CoralManipulator(CoralManipulatoIO io) {
    this.io = io;
  }

  @AutoLogOutput(key = "CoralManipulator/State")
  @Getter
  private RollersState rollersState = RollersState.IDLE;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
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
