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
  // private final ArmVisualizer visualizer = new ArmVisualizer();
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", gains.kD());
  // private boolean characterizing;
  private boolean brakeModeEnabled;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  @AutoLogOutput(key = "Superstructure/Arm/Position") private DoubleSupplier positionSetpoint = ArmState.STOW.getArmAngle();
  private Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kRising);
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
    INTAKING(new LoggedTunableNumber("Superstructure/Arm/INTAKING", -.1)),
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
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Superstructure/Arm", inputs);
    if (disableSupplier.getAsBoolean()) {
      io.stop();
    }

    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);

    // io.setBrakeMode(!coastSupplier.getAsBoolean() || armState == ArmState.STOW);

    // if (!disableSupplier.getAsBoolean()) {
    //   io.setArmPosition(positionSetpoint);
    // }
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {

    return atGoalDebouncer.calculate(
        EqualsUtil.epsilonEquals(
            inputs.positionRad, positionSetpoint.getAsDouble(), ARM_TOLERANCE));
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

  public void setPosition(DoubleSupplier positionSetpoint) {
    if (this.positionSetpoint != positionSetpoint) {
      this.positionSetpoint = positionSetpoint;
      io.setArmPosition(positionSetpoint.getAsDouble());
    }
  }
}
