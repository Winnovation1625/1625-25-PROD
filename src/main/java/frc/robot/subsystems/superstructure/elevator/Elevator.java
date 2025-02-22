package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator{

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  //  private final ElevatorVisualizer visualizer =
  //   new ElevatorVisualizer(ElevatorConstants.elevatorPose);

  // private static final LoggedTunableNumber kP =
  //   new LoggedTunableNumber("Elevator/kP", gains.kP());
  // private static final LoggedTunableNumber kI =
  //   new LoggedTunableNumber("Elevator/kI", gains.kI());
  // private static final LoggedTunableNumber kD =
  //   new LoggedTunableNumber("Elevator/kD", gains.kD());
  // private static final LoggedTunableNumber kS =
  //   new LoggedTunableNumber("Elevator/kS", gains.ffkS());
  // private static final LoggedTunableNumber kV =
  //   new LoggedTunableNumber("Elevator/kV", gains.ffkV());
  // private static final LoggedTunableNumber kA =
  //   new LoggedTunableNumber("Elevator/kA", gains.ffkA());
  // private static final LoggedTunableNumber kG =
  //   new LoggedTunableNumber("Elevator/kG", gains.ffkG());
  // private static final LoggedTunableNumber cruiseV =
  //   new LoggedTunableNumber("Elevator/cruiseV", cruiseVelocity);
  // private static final LoggedTunableNumber cruiseA =
  //   new LoggedTunableNumber("Elevator/cruiseA", cruiseAcceleration);
  // private static final LoggedTunableNumber cruiseJ =
  //   new LoggedTunableNumber("Elevator/cruiseJ", cruiseJerk);

  @RequiredArgsConstructor
  public enum ElevatorState {
    STOP(() -> 0),
    STOW(new LoggedTunableNumber("Superstructure/Elevator/Stow", 0)),
    LEVEL_0NE(new LoggedTunableNumber("Superstructure/Elevator/L1", 12)),
    LEVEL_TWO(new LoggedTunableNumber("Superstructure/Elevator/L2", 22)),
    LEVEL_THREE(new LoggedTunableNumber("Superstructure/Elevator/L3", 33)),
    LEVEL_FOUR(new LoggedTunableNumber("Superstructure/Elevator/L4", 44)),
    BARGE(new LoggedTunableNumber("Superstructure/Elevator/Barge", 55));

    @Getter private final DoubleSupplier elevatorSetpointFromGround;
  }

  @AutoLogOutput(key = "Superstructure/Elevator/ElevatorState")
  // @Getter
  // @Setter
  private ElevatorState elevatorState = ElevatorState.STOW;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  private boolean characterizing;
  private boolean brakeModeEnabled;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Superstructure/Elevator", inputs);

    if (disableSupplier.getAsBoolean() || elevatorState == ElevatorState.STOP) {
      io.stop();
    }

    setBrakeMode(!coastSupplier.getAsBoolean() || DriverStation.isEnabled());

    // visualizer.updateVisualizer(inputs.positionRads);

    if (!characterizing
        && brakeModeEnabled
        && !disableSupplier.getAsBoolean()
        && elevatorState != ElevatorState.STOP) {

      Logger.recordOutput(
          "Superstructure/Elevator/ElevatorSetpoint",
          Units.radiansToDegrees(elevatorState.getElevatorSetpointFromGround().getAsDouble()));
    }
  }

  @AutoLogOutput(key = "Superstructure/Elevator/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        inputs.positionRad,
        elevatorState.getElevatorSetpointFromGround().getAsDouble(),
        ELEVATOR_TOLERANCE_METERS);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void runCharacterization(double amps) {
    characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }

  public void changeLevel() {
    elevatorState = ElevatorState.LEVEL_0NE;
    io.setPosition(ElevatorState.LEVEL_0NE.getElevatorSetpointFromGround().getAsDouble());
  }

  public void tempStop() {
    io.stop();
  }
}
