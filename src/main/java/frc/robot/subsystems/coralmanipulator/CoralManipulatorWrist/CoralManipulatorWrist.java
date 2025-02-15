package frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist;

import static frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist.CoralManipulatorWristConstants.*;

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
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class CoralManipulatorWrist {

  private final CoralManipulatorWristIO io;
  private final CoralManipulatorWristIOInputsAutoLogged inputs =
      new CoralManipulatorWristIOInputsAutoLogged();
  // private final IntakeArmVisualizer visualizer =
  //   new IntakeArmVisualizer(IntakeArmConstants.intakePose);
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("CoralManipulatorWrist/kP",
  gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("CoralManipulatorWrist/kI",
  gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("CoralManipulatorWrist/kD",
  gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("CoralManipulatorWrist/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("CoralManipulatorWrist/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("CoralManipulatorWrist/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("CoralManipulatorWrist/kG", gains.ffkG());
  private static final LoggedTunableNumber cruiseV =
      new LoggedTunableNumber("CoralManipulatorWrist/cruiseV", cruiseVelocity);
  private static final LoggedTunableNumber cruiseA =
      new LoggedTunableNumber("CoralManipulatorWrist/cruiseA", cruiseAcceleration);
  private static final LoggedTunableNumber cruiseJ =
      new LoggedTunableNumber("CoralManipulatorWrist/cruiseJ", cruiseJerk);
  private static final LoggedNetworkBoolean coastSupplier = 
      new LoggedNetworkBoolean("CoralManipulatorWrist/coastSupplier", false);

  public CoralManipulatorWrist(CoralManipulatorWristIO io) {
    this.io = io;
  }

  @AutoLogOutput @Getter @Setter private WristState wristState = WristState.STOW;
  private boolean characterizing;
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;

  @RequiredArgsConstructor
  public enum WristState {
    STOP(() -> 0),
    STOW(new LoggedTunableNumber("Wrist/Stow", 5)),
    TROUGH(new LoggedTunableNumber("Wrist/Trough", 10)),
    LEVELONE(new LoggedTunableNumber("Wrist/LevelOne", 20)),
    LEVELTWO(new LoggedTunableNumber("Wrist/LevelTwo", 30)),
    LEVELTHREE(new LoggedTunableNumber("Wrist/LevelThree", 40));

    private final DoubleSupplier wristSetpointSupplier;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralManipulatorWrist", inputs);
    // Check if disabled
    if (disableSupplier.getAsBoolean() || wristState == wristState.STOP) {
      io.stop();
    }
    // LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    // LoggedTunableNumber.ifChanged(
    //     hashCode(), kSVA -> io.setFF(kSVA[0], kSVA[1], kSVA[2], kSVA[3]), kS, kV, kA, kG);
    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     kSVA -> io.setMotionMagicCruise(kSVA[0], kSVA[1], kSVA[2]),
    //     cruiseV,
    //     cruiseA,
    //     cruiseJ);
    // visualizer.updateVisualizer(inputs.positionRads);
    if (wristState == WristState.STOW && atGoal()) {
      io.stop();
    }
    if (!characterizing
        && !disableSupplier.getAsBoolean()
        && wristState != WristState.STOP) {

      io.setPosition(wristState.wristSetpointSupplier);

    }

  }

  @AutoLogOutput(key = "Superstructure/CoralManiuplatorWrist/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(inputs.positionRad, wristState.wristSetpointSupplier.getAsDouble(), CoralManipulatorWristConstants.armTolerance);
  }

  public void setBrakeMode(boolean coastSupplier) {
    setBrakeMode(coastSupplier);
  }


}
