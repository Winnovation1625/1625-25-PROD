package frc.robot.subsystems.superstructure.arm;

import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.EqualsUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

  public Arm(ArmIO io) {
    this.io = io;
  }

  // private boolean characterizing;
  private boolean brakeModeEnabled;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private DoubleSupplier positionSetpoint;

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Superstructre/Arm", inputs);
    if (disableSupplier.getAsBoolean()) {
      io.stop();
    }

    // LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI,
    // kD);

    // io.setBrakeMode(!coastSupplier.getAsBoolean() || armState == ArmState.STOW);

    if (!disableSupplier.getAsBoolean()) {
      io.setArmPosition(positionSetpoint);
    }
  }

  @AutoLogOutput(key = "Superstructre/Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        inputs.positionRad, positionSetpoint.getAsDouble(), ARM_TOLERANCE);
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
    this.positionSetpoint = positionSetpoint;
  }

}
