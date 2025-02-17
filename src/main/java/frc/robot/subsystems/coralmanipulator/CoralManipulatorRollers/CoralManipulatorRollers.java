package frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralManipulatorRollers extends SubsystemBase {

  private final CoralManipulatorRollersIO io;
  private final CoralManipulatorRollersIOInputsAutoLogged inputs =
      new CoralManipulatorRollersIOInputsAutoLogged();

  @RequiredArgsConstructor
  public enum RollersState {
    INTAKING(new LoggedTunableNumber("CoralManipulatorRollers/Intaking", 10)),
    SHOOTING(new LoggedTunableNumber("CoralManipulatorRollers/Shooting", 10)),
    EJECTING(new LoggedTunableNumber("CoralManipulatorRollers/Ejecting", 20)),
    IDLE(() -> 0);

    private final DoubleSupplier voltageSupplier;
  }

  public CoralManipulatorRollers(CoralManipulatorRollersIO io) {
    this.io = io;
  }

  @AutoLogOutput(key = "CoralManipulatorRollers/State")
  @Getter
  private RollersState rollersState = RollersState.IDLE;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralManipulatorRollers", inputs);

    if (DriverStation.isDisabled()) {
      rollersState = RollersState.IDLE;
    }

    io.setVoltage(rollersState.voltageSupplier);
  }

  public Command setDesiredStateCommand(RollersState goal) {
    return startEnd(() -> this.rollersState = goal, () -> this.rollersState = RollersState.IDLE)
        .withName("CoralManipulatorRollers " + goal);
  }
}
