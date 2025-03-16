package frc.robot.subsystem.superstructure.elevator;

import static frc.robot.subsystem.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  //  private final ElevatorVisualizer visualizer =
  //   new ElevatorVisualizer(ElevatorConstants.elevatorPose);

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/kP", positionGains.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/kI", positionGains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/kD", positionGains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/kS", positionGains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", positionGains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/kA", positionGains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/kG", positionGains.ffkG());
  private static final LoggedTunableNumber cruiseV =
      new LoggedTunableNumber("Elevator/cruiseV", positionGains.cruiseV());
  private static final LoggedTunableNumber cruiseA =
      new LoggedTunableNumber("Elevator/cruiseA", positionGains.cruiseA());
  private static final LoggedTunableNumber cruiseJ =
      new LoggedTunableNumber("Elevator/cruiseJ", positionGains.cruiseJ());
  private static final LoggedTunableNumber differentialKP =
      new LoggedTunableNumber("Elevator/Differential/kP", followerGains.kP());
  private static final LoggedTunableNumber differentialKI =
      new LoggedTunableNumber("Elevator/Differential/kI", followerGains.kI());
  private static final LoggedTunableNumber differentialKD =
      new LoggedTunableNumber("Elevator/Differential/kD", followerGains.kD());
  private static final LoggedTunableNumber differentialKS =
      new LoggedTunableNumber("Elevator/Differential/kS", followerGains.ffkS());
  private static final LoggedTunableNumber differentialKV =
      new LoggedTunableNumber("Elevator/Differential/kV", followerGains.ffkV());
  private static final LoggedTunableNumber differentialKA =
      new LoggedTunableNumber("Elevator/Differential/kA", followerGains.ffkA());
  private static final LoggedTunableNumber differentialKG =
      new LoggedTunableNumber("Elevator/Differential/kG", followerGains.ffkG());
  private static final LoggedTunableNumber differentialCruiseV =
      new LoggedTunableNumber("Elevator/Differential/cruiseV", followerGains.cruiseV());
  private static final LoggedTunableNumber differentialCruiseA =
      new LoggedTunableNumber("Elevator/Differential/cruiseA", followerGains.cruiseA());
  private static final LoggedTunableNumber differentialCruiseJ =
      new LoggedTunableNumber("Elevator/Differential/cruiseJ", followerGains.cruiseJ());
  private static final LoggedTunableNumber elevatorTolerance =
      new LoggedTunableNumber("Elevator/ToleranceMeters", ELEVATOR_TOLERANCE_METERS);
  private boolean characterizing;
  private boolean brakeModeEnabled;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;

  @AutoLogOutput(key = "Superstructure/Elevator/Position")
  private DoubleSupplier setpoint = () -> 0;

  private Debouncer atGoalDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  @RequiredArgsConstructor
  public enum ElevatorState {
    ALGAE_INTAKING(
        new LoggedTunableNumber(
            "Superstructure/Elevator/ALGAE_INTAKING", Units.inchesToMeters(19.685))),
    CORAL_INTAKING(
        new LoggedTunableNumber(
            "Superstructure/Elevator/CORAL_INTAKING", Units.inchesToMeters(10))),
    STOW(new LoggedTunableNumber("Superstructure/Elevator/STOW", Units.inchesToMeters(27.5591))),
    ALGAE_STOW(
        new LoggedTunableNumber("Superstructure/Elevator/ALGAE STOW", Units.inchesToMeters(34))),
    ALGAE_CLEARANCE(
        new LoggedTunableNumber(
            "Superstructure/Elevator/ALGAE CLEARANCE", Units.inchesToMeters(47))),
    CLIMB(new LoggedTunableNumber("Superstructure/Elevator/CLIMB", Units.inchesToMeters(27.5591))),
    BARGE(new LoggedTunableNumber("Superstructure/Elevator/BARGE", Units.inchesToMeters(84.75))),
    CLVL2(new LoggedTunableNumber("Superstructure/Elevator/CLVL2", Units.inchesToMeters(38.38583))),
    TROUGH(new LoggedTunableNumber("Superstructure/Elevator/TROUGH", Units.inchesToMeters(27.107))),
    CLVL3(new LoggedTunableNumber("Superstructure/Elevator/CLVL3", Units.inchesToMeters(55.1181))),
    CLVL4(new LoggedTunableNumber("Superstructure/Elevator/CLVL4", Units.inchesToMeters(59.80315))),
    ALVL2(new LoggedTunableNumber("Superstructure/Elevator/ALVL2", Units.inchesToMeters(30))),
    ALVL3(new LoggedTunableNumber("Superstructure/Elevator/ALVL3", Units.inchesToMeters(35))),
    PROCESS(new LoggedTunableNumber("Superstructure/Elevator/PROCESS", Units.inchesToMeters(13))),
    STOP(() -> 0);
    @Getter private final DoubleSupplier elevatorHeight;
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Superstructure/Elevator", inputs);

    if (disableSupplier.getAsBoolean()) {
      io.stop();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid ->
            io.setPositionPID(
                pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8], pid[9]),
        kP,
        kI,
        kD,
        kG,
        kS,
        kV,
        kA,
        cruiseA,
        cruiseV,
        cruiseJ);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid ->
            io.setFollowerPID(
                pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8], pid[9]),
        differentialKP,
        differentialKI,
        differentialKD,
        differentialKG,
        differentialKS,
        differentialKV,
        differentialKA,
        differentialCruiseA,
        differentialCruiseV,
        differentialCruiseJ);

    setBrakeMode(!coastSupplier.getAsBoolean() || DriverStation.isEnabled());

    // visualizer.updateVisualizer(inputs.positionRads);

    if (!characterizing && !disableSupplier.getAsBoolean()) {
      // io.setPosition(convertDistanceMetersToRadians(positionSetpoint));
      Logger.recordOutput(
          "Superstructure/Elevator/ElevatorSetpointInches",
          Units.metersToInches(setpoint.getAsDouble()));
      Logger.recordOutput(
          "Superstructure/Elevator/ElevatorSetpointRads",
          convertDistanceMetersToRadians(setpoint.getAsDouble()));
    }
    Logger.recordOutput(
        "Superstructure/Elevator/LeftElevatorPositionMeters",
        convertRadiansToDistanceMeters(inputs.positionRad));
    Logger.recordOutput(
        "Superstructure/Elevator/RightElevatorPositionMeters",
        convertRadiansToDistanceMeters(inputs.positionRotationsFollower));
  }

  @AutoLogOutput(key = "Superstructure/Elevator/AtGoal")
  public boolean atGoal() {
    return atGoalDebouncer.calculate(
        EqualsUtil.epsilonEquals(
            convertRadiansToDistanceMeters(inputs.positionRad),
            setpoint.getAsDouble(),
            elevatorTolerance.get()));
  }

  public void setPosition(DoubleSupplier positionSetpointMeters) {
    System.out.println("Set Command For Elevator Ran to " + positionSetpointMeters.getAsDouble());
    if (setpoint.getAsDouble() != positionSetpointMeters.getAsDouble()) {
      this.setpoint = positionSetpointMeters;
      io.setPosition(convertDistanceMetersToRadians(positionSetpointMeters.getAsDouble()));
    }
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
