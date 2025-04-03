package frc.robot.subsystem.superstructure.arm;

import static frc.robot.subsystem.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystem.manipulator.Manipulator.GamepieceState;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
  private Supplier<GamepieceState> gamepieceStateSupplier;

  @AutoLogOutput(key = "Superstructure/Arm/Position")
  private ArmState positionSetpoint = ArmState.STOP;

  private Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private boolean characterizing;
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", gains.ffkS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", gains.ffkV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", gains.ffkA());
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", gains.ffkG());
  private static final LoggedTunableNumber kG_Algae =
      new LoggedTunableNumber("Arm/kG_Algae", gains.ffkG_Algae());
  private static final LoggedTunableNumber kG_Coral =
      new LoggedTunableNumber("Arm/kG_Coral", gains.ffkG_Coral());
  private static final LoggedTunableNumber cruiseV =
      new LoggedTunableNumber("Arm/cruiseV", gains.cruiseVelocity());
  private static final LoggedTunableNumber cruiseA =
      new LoggedTunableNumber("Arm/cruiseA", gains.cruiseAcceleration());
  @AutoLogOutput private double kGCurrent = kG.get();

  @RequiredArgsConstructor
  public enum ArmState {
    ALGAE_INTAKING(new LoggedTunableNumber("Superstructure/Arm/INTAKING", -0.5)),
    CORAL_INTAKING(new LoggedTunableNumber("Superstructure/Arm/CORAL_INTAKING", 0)),
    STOW(new LoggedTunableNumber("Superstructure/Arm/STOW", Math.PI / 2)),
    CLIMB(new LoggedTunableNumber("Superstructure/Arm/CLIMB", Math.PI / 2)),
    BARGE(new LoggedTunableNumber("Superstructure/Arm/BARGE", 0.83)),
    CLVL2(new LoggedTunableNumber("Superstructure/Arm/CLVL2", 0.32)),
    TROUGH(new LoggedTunableNumber("Superstructure/Arm/TROUGH", 0)),
    CLVL3(new LoggedTunableNumber("Superstructure/Arm/CLVL3", 0.32)),
    CLVL4(new LoggedTunableNumber("Superstructure/Arm/CLVL4", 0.69)),
    ALVL2(new LoggedTunableNumber("Superstructure/Arm/ALVL2", 0.38)),
    ALVL3(new LoggedTunableNumber("Superstructure/Arm/ALVL3", 0.1)),
    PROCESS(new LoggedTunableNumber("Superstructure/Arm/PROCESS", 0.05)),
    FLAT(() -> 0),
    STOP(() -> 0);
    @Getter private final DoubleSupplier armAngle;
  }

  public Arm(ArmIO io, Supplier<GamepieceState> gamepieceStateSupplier) {
    this.io = io;
    io.setBrakeMode(true);
    io.setInverted(ARM_INVERTED);
    this.gamepieceStateSupplier = gamepieceStateSupplier;
  }

  public void periodic() {
    io.updateInputs(inputs);
    GamepieceState gamepieceState = gamepieceStateSupplier.get();

    Logger.processInputs("Superstructure/Arm", inputs);
    if (disableSupplier.getAsBoolean()) {
      positionSetpoint = ArmState.STOP;
      io.stop();
    }
    Logger.recordOutput(
        "Superstructure/Arm/SetpointAngleRads", positionSetpoint.getArmAngle().getAsDouble());
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setPID(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8], 0),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG,
        cruiseV,
        cruiseA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setPID(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8], 1),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG_Coral,
        cruiseV,
        cruiseA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setPID(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8], 2),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG_Algae,
        cruiseV,
        cruiseA);
    // LoggedNetworkBoolean.ifChanged(hashCode(), brakeEnabled -> io.setBrakeMode(brakeModeEnabled),
    // brakeEnabled);
    // io.setBrakeMode(!coastSupplier.getAsBoolean() || armState == ArmState.STOW);

    // if (!disableSupplier.getAsBoolean()) {
    //   io.setArmPosition(positionSetpoint);
    // }

    // if (gamepieceState == GamepieceState.NONE && kGCurrent != kG.get()) {
    //   io.changeKG(kG.get());
    //   kGCurrent = kG.get();
    // } else if ((gamepieceState == GamepieceState.CORAL_IN_MANIPULATOR
    //         || gamepieceState == GamepieceState.CORAL_STAGING)
    //     && kGCurrent != kG_Coral.get()) {
    //   io.changeKG(kG_Coral.get());
    //   kGCurrent = kG_Coral.get();
    // } else if (gamepieceState == GamepieceState.ALGAE_IN_CLAW && kGCurrent != kG_Algae.get()) {
    //   io.changeKG(kG_Algae.get());
    //   kGCurrent = kG_Algae.get();
    // }
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {

    return positionSetpoint == ArmState.STOP
        || atGoalDebouncer.calculate(
            EqualsUtil.epsilonEquals(
                inputs.positionRad,
                positionSetpoint.getArmAngle().getAsDouble(),
                armTolerance.get()));
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

  public void setPosition(ArmState positionSetpoint) {
    System.out.println(
        "Set Command For Arm Ran to " + positionSetpoint.getArmAngle().getAsDouble());
    if (this.positionSetpoint != positionSetpoint) {
      this.positionSetpoint = positionSetpoint;
      io.setArmPosition(positionSetpoint.getArmAngle().getAsDouble(), gamepieceStateSupplier.get());
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
