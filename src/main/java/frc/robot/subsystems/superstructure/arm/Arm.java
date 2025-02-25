package frc.robot.subsystems.superstructure.arm;

import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

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

public class Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  // private final ArmVisualizer visualizer = new ArmVisualizer();
  // private static final LoggedTunableNumber kP =
  //     new LoggedTunableNumber("Arm/kP", gains.kP());
  // private static final LoggedTunableNumber kI =
  //     new LoggedTunableNumber("Arm/kI", gains.kI());
  // private static final LoggedTunableNumber kD =
  //     new LoggedTunableNumber("Arm/kD", gains.kD());
  // private static final LoggedTunableNumber kS =
  //     new LoggedTunableNumber("Arm/kS", gains.ffkS());
  // private static final LoggedTunableNumber kV =
  //     new LoggedTunableNumber("Arm/kV", gains.ffkV());
  // private static final LoggedTunableNumber kA =
  //     new LoggedTunableNumber("Arm/kA", gains.ffkA());
  // private static final LoggedTunableNumber kG =
  //     new LoggedTunableNumber("Arm/kG", gains.ffkG());
  // private static final LoggedTunableNumber cruiseV =
  //     new LoggedTunableNumber("Arm/cruiseV", cruiseVelocity);
  // private static final LoggedTunableNumber cruiseA =
  //     new LoggedTunableNumber("Arm/cruiseA", cruiseAcceleration);
  // private static final LoggedTunableNumber cruiseJ =
  //     new LoggedTunableNumber("Arm/cruiseJ", cruiseJerk);
  @RequiredArgsConstructor
  public enum ArmState {
    STOW(new LoggedTunableNumber("Superstructure/Arm/STOW", Math.PI / 2)),
    ALVL2(new LoggedTunableNumber("Superstructure/Arm/ALGL", .2)),
    ALVL3(new LoggedTunableNumber("Superstructure/Arm/ALVL3", .3)),
    CLVL1(new LoggedTunableNumber("Superstructure/Arm/CLVL1", 2)),
    CLVL2(new LoggedTunableNumber("Superstructure/Arm/CLVL2", .5)),
    CLVL3(new LoggedTunableNumber("Superstructure/Arm/CLVL3", .6)),
    CLVL4(new LoggedTunableNumber("Superstructure/Arm/CLVL4", .7)),
    INTAKE(new LoggedTunableNumber("Superstructure/Arm/INTAKE", .8)),
    PROCESS(new LoggedTunableNumber("Superstructure/Arm/PROCESS", .9)),
    BARGE(new LoggedTunableNumber("Superstructure/Arm/BARGE", 1)),
    STOP(() -> 0);

    @Getter private final DoubleSupplier armSetpointSupplier;
  }

  @AutoLogOutput(key = "Superstructure/Arm/ArmState")
  @Getter
  @Setter
  private ArmState armState = ArmState.STOW;

  public Arm(ArmIO io) {
    this.io = io;
  }

  // private boolean characterizing;
  private boolean brakeModeEnabled;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Superstructre/Arm", inputs);
    if (disableSupplier.getAsBoolean() || armState == ArmState.STOP) {
      io.stop();
    }

    // LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI,
    // kD);

    // io.setBrakeMode(!coastSupplier.getAsBoolean() || armState == ArmState.STOW);

    if (!disableSupplier.getAsBoolean() && armState != ArmState.STOP) {
      io.setArmPosition(armState.armSetpointSupplier);
    }
  }

  @AutoLogOutput(key = "Superstructre/Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        inputs.positionRad, armState.armSetpointSupplier.getAsDouble(), ARM_TOLERANCE);
  }

  public void setBreakMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(enabled);
  }

  public void runCharacterizaiton(double amps) {
    // characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void endCharacterization() {
    // characterizing = false;
  }

  public double getArmPos() {
    return inputs.positionRad;
  }
}
