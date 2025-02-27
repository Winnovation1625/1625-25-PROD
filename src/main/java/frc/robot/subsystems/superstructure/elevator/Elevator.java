package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  //  private final ElevatorVisualizer visualizer =
  //   new ElevatorVisualizer(ElevatorConstants.elevatorPose);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", gains.kD());
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
    STOW(new LoggedTunableNumber("Superstructure/Elevator/STOW", Units.inchesToMeters(27.5591))),
    ALVL2(new LoggedTunableNumber("Superstructure/Elevator/ALGL", Units.inchesToMeters(30))),
    ALVL3(new LoggedTunableNumber("Superstructure/Elevator/ALVL3", Units.inchesToMeters(35))),
    CLVL1(new LoggedTunableNumber("Superstructure/Elevator/CLVL1", Units.inchesToMeters(40.107))),
    CLVL2(new LoggedTunableNumber("Superstructure/Elevator/CLVL2", Units.inchesToMeters(39.3701))),
    CLVL3(new LoggedTunableNumber("Superstructure/Elevator/CLVL3", Units.inchesToMeters(55.1181))),
    CLVL4(new LoggedTunableNumber("Superstructure/Elevator/CLVL4", Units.inchesToMeters(84.80315))),
    INTAKE(new LoggedTunableNumber("Superstructure/Elevator/INTAKE", Units.inchesToMeters(15))),
    PROCESS(new LoggedTunableNumber("Superstructure/Elevator/PROCESS", Units.inchesToMeters(13))),
    BARGE(new LoggedTunableNumber("Superstructure/Elevator/BARGE", Units.inchesToMeters(80))),
    STOP(() -> 0);

    @Getter private final DoubleSupplier elevatorSetpointFromGround;
  }

  @AutoLogOutput(key = "Superstructure/Elevator/ElevatorState")
  @Getter
  @Setter
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

    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);

    setBrakeMode(!coastSupplier.getAsBoolean() || DriverStation.isEnabled());

    // visualizer.updateVisualizer(inputs.positionRads);

    if (!characterizing
        && brakeModeEnabled
        && !disableSupplier.getAsBoolean()
        && elevatorState != ElevatorState.STOP) {
      io.setPosition(
          convertDistanceMetersToRadians(elevatorState.elevatorSetpointFromGround.getAsDouble()));
      Logger.recordOutput(
          "Superstructure/Elevator/ElevatorSetpointInches",
          Units.metersToInches(elevatorState.getElevatorSetpointFromGround().getAsDouble()));
      Logger.recordOutput(
          "Superstructure/Elevator/ElevatorSetpointRads",
          convertDistanceMetersToRadians(
              elevatorState.getElevatorSetpointFromGround().getAsDouble()));
    }
  }

  @AutoLogOutput(key = "Superstructure/Elevator/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        convertRadiansToDistanceMeters(inputs.positionRad),
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

  public static double convertDistanceMetersToRadians(double distanceMeters) {
    return distanceMeters / ELEVATOR_DRUM_RADIUS;
  }

  public static double convertRadiansToDistanceMeters(double posRads) {
    return posRads * ELEVATOR_DRUM_RADIUS;
  }

  public double getElevatorHeight() {
    return convertRadiansToDistanceMeters(inputs.positionRad);
  }
}
