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
  private static final LoggedTunableNumber elevatorTolerance =
      new LoggedTunableNumber("Elevator/ToleranceMeters", ELEVATOR_TOLERANCE_METERS);
  private boolean characterizing;
  private boolean brakeModeEnabled;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;

  @AutoLogOutput(key = "Superstructure/Elevator/Position")
  private ElevatorState setpoint = ElevatorState.STOP;

  private Debouncer atGoalDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  @RequiredArgsConstructor
  public enum ElevatorState {
    ALGAE_INTAKING(new LoggedTunableNumber("Superstructure/Elevator/ALGAE_INTAKING", 0.21)),
    CORAL_INTAKING(new LoggedTunableNumber("Superstructure/Elevator/CORAL_INTAKING", 0.295)),
    STOW(new LoggedTunableNumber("Superstructure/Elevator/STOW", 0.285)),
    ALGAE_STOW(new LoggedTunableNumber("Superstructure/Elevator/ALGAE STOW", 0.3)),
    ALGAE_CLEARANCE(new LoggedTunableNumber("Superstructure/Elevator/ALGAE CLEARANCE", 0.6)),
    CLIMB(new LoggedTunableNumber("Superstructure/Elevator/CLIMB", Units.inchesToMeters(27.5591))),
    BARGE(new LoggedTunableNumber("Superstructure/Elevator/BARGE", 1.63)),
    CLVL2(new LoggedTunableNumber("Superstructure/Elevator/CLVL2", 0.55)),
    TROUGH(new LoggedTunableNumber("Superstructure/Elevator/TROUGH", 0.26)),
    CLVL3(new LoggedTunableNumber("Superstructure/Elevator/CLVL3", 0.94)),
    CLVL4(new LoggedTunableNumber("Superstructure/Elevator/CLVL4", 1.755)),
    ALVL2(new LoggedTunableNumber("Superstructure/Elevator/ALVL2", 0.41)),
    ALVL3(new LoggedTunableNumber("Superstructure/Elevator/ALVL3", 0.67)),
    PROCESS(new LoggedTunableNumber("Superstructure/Elevator/PROCESS", 0.18)),
    STOP(() -> 0);
    @Getter private final DoubleSupplier elevatorHeight;
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Superstructure/Elevator", inputs);
    Logger.recordOutput(
        "Superstructure/Elevator/SetpointHeightMeters", setpoint.getElevatorHeight());

    if (disableSupplier.getAsBoolean()) {
      setpoint = ElevatorState.STOP;
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

    setBrakeMode(!coastSupplier.getAsBoolean() || DriverStation.isEnabled());

    // visualizer.updateVisualizer(inputs.positionRads);

    if (!characterizing && !disableSupplier.getAsBoolean()) {
      // io.setPosition(convertDistanceMetersToRadians(positionSetpoint));
      Logger.recordOutput(
          "Superstructure/Elevator/ElevatorSetpointInches",
          Units.metersToInches(setpoint.getElevatorHeight().getAsDouble()));
      Logger.recordOutput(
          "Superstructure/Elevator/ElevatorSetpointRads",
          convertDistanceMetersToRadians(setpoint.getElevatorHeight().getAsDouble()));
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
        setpoint == ElevatorState.STOP
            || EqualsUtil.epsilonEquals(
                convertRadiansToDistanceMeters(inputs.positionRad),
                setpoint.getElevatorHeight().getAsDouble(),
                elevatorTolerance.get()));
  }

  public void setPosition(ElevatorState positionSetpoint) {
    System.out.println(
        "Set Command For Elevator Ran to " + positionSetpoint.getElevatorHeight().getAsDouble());
    if (setpoint.getElevatorHeight().getAsDouble()
        != positionSetpoint.getElevatorHeight().getAsDouble()) {
      this.setpoint = positionSetpoint;
      io.setPosition(
          convertDistanceMetersToRadians(positionSetpoint.getElevatorHeight().getAsDouble()));
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

  public void setEncoderPosition(double encoderSetpoint) {
    io.setEncoderPosition(encoderSetpoint);
  }
}
