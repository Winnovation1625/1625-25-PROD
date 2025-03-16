package frc.robot.subsystem.superstructure.arm;

import static frc.robot.subsystem.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", gains.kD());
  private static final LoggedTunableNumber armTolerance =
      new LoggedTunableNumber("Arm/Tolerance", ARM_TOLERANCE);
  private boolean brakeModeEnabled;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;

  @AutoLogOutput(key = "Superstructure/Arm/Position")
  private DoubleSupplier positionSetpoint = ArmState.STOP.getArmAngle();

  private Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private boolean characterizing;
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", gains.ffkS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", gains.ffkV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", gains.ffkA());
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", gains.ffkG());
  private static final LoggedTunableNumber cruiseV =
      new LoggedTunableNumber("Arm/cruiseV", gains.cruiseVelocity());
  private static final LoggedTunableNumber cruiseA =
      new LoggedTunableNumber("Arm/cruiseA", gains.cruiseAcceleration());

  @RequiredArgsConstructor
  public enum ArmState {
    ALGAE_INTAKING(new LoggedTunableNumber("Superstructure/Arm/INTAKING", -.2)),
    CORAL_INTAKING(new LoggedTunableNumber("Superstructure/Arm/CORAL_INTAKING", 0)),
    STOW(new LoggedTunableNumber("Superstructure/Arm/STOW", Math.PI / 2)),
    CLIMB(new LoggedTunableNumber("Superstructure/Arm/CLIMB", Math.PI / 2)),
    BARGE(new LoggedTunableNumber("Superstructure/Arm/BARGE", 1)),
    CLVL2(new LoggedTunableNumber("Superstructure/Arm/CLVL2", .5)),
    TROUGH(new LoggedTunableNumber("Superstructure/Arm/TROUGH", .3)),
    CLVL3(new LoggedTunableNumber("Superstructure/Arm/CLVL3", .6)),
    CLVL4(new LoggedTunableNumber("Superstructure/Arm/CLVL4", .7)),
    ALVL2(new LoggedTunableNumber("Superstructure/Arm/ALVL2", .2)),
    ALVL3(new LoggedTunableNumber("Superstructure/Arm/ALVL3", .3)),
    PROCESS(new LoggedTunableNumber("Superstructure/Arm/PROCESS", .9)),
    STOP(() -> 0);
    @Getter private final DoubleSupplier armAngle;
  }

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);
    io.setInverted(ARM_INVERTED);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Superstructure/Arm", inputs);
    if (disableSupplier.getAsBoolean()) {
      io.stop();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setPID(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8]),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG,
        cruiseV,
        cruiseA);
    // LoggedNetworkBoolean.ifChanged(hashCode(), brakeEnabled -> io.setBrakeMode(brakeModeEnabled),
    // brakeEnabled);
    // io.setBrakeMode(!coastSupplier.getAsBoolean() || armState == ArmState.STOW);

    // if (!disableSupplier.getAsBoolean()) {
    //   io.setArmPosition(positionSetpoint);
    // }
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {

    return atGoalDebouncer.calculate(
        EqualsUtil.epsilonEquals(
            inputs.positionRad, positionSetpoint.getAsDouble(), armTolerance.get()));
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(enabled);
  }

  public void runCharacterizaiton(double amps) {
    characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }

  public double getArmPos() {
    return inputs.positionRad;
  }

  public void setPosition(DoubleSupplier positionSetpoint) {
    System.out.println("Set Command For Arm Ran to " + positionSetpoint.getAsDouble());
    if (this.positionSetpoint.getAsDouble() != positionSetpoint.getAsDouble()) {
      this.positionSetpoint = positionSetpoint;
      io.setArmPosition(positionSetpoint.getAsDouble());
    }
  }

  // public Command upStaticCharacterization() {
  //   final StaticCharacterizationState state = new StaticCharacterizationState();
  //   Timer timer = new Timer();
  //   return Commands.startRun(
  //           () -> {
  //             stopProfile = true;
  //             timer.restart();
  //           },
  //           () -> {
  //             stopProfile = true;
  //             state.characterizationOutput = characterizationRampRate.get() * timer.get();
  //             io.runOpenLoop(state.characterizationOutput);
  //             Logger.recordOutput(
  //                 "Elevator/CharacterizationOutputUp", state.characterizationOutput);
  //           })
  //       .until(() -> inputs.data.velocityRadPerSec() >= characterizationUpVelocityThresh.get())
  //       .andThen(io::stop)
  //       .andThen(Commands.idle())
  //       .finallyDo(
  //           () -> {
  //             stopProfile = false;
  //             timer.stop();
  //             Logger.recordOutput(
  //                 "Elevator/CharacterizationOutputUp", state.characterizationOutput);
  //           });
  // }

  // public Command downStaticCharacterization() {
  //   final StaticCharacterizationState state = new StaticCharacterizationState();
  //   Timer timer = new Timer();
  //   return Commands.startRun(
  //           () -> {
  //             stopProfile = true;
  //             timer.restart();
  //           },
  //           () -> {
  //             state.characterizationOutput =
  //                 characterizationDownStartAmps.get()
  //                     - characterizationRampRate.get() * timer.get();
  //             io.runOpenLoop(state.characterizationOutput);
  //             Logger.recordOutput(
  //                 "Elevator/CharacterizationOutputDown", state.characterizationOutput);
  //           })
  //       .until(() -> inputs.data.velocityRadPerSec() <= characterizationDownVelocityThresh.get())
  //       .andThen(io::stop)
  //       .andThen(Commands.idle())
  //       .finallyDo(
  //           () -> {
  //             stopProfile = false;
  //             timer.stop();
  //             Logger.recordOutput("Arm/CharacterizationOutputDown",
  // state.characterizationOutput);
  //           });
  // }

  // private static class StaticCharacterizationState {
  //   public double characterizationOutput = 0.0;
  // }
}
