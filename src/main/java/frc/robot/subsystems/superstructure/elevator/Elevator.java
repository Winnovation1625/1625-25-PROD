package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.EqualsUtil;

public class Elevator {

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

  //@RequiredArgsConstructor
  public enum ElevatorState {
    STOP(), // Will discuss about values.
    STOW(),
    LEVEL3(),
    LEVEL4(),
    BARGE();

    private DoubleSupplier elevatorSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(elevatorSetpointSupplier.getAsDouble());
    }
  }

  @AutoLogOutput(key = "Superstructure/ElevatorArm/ElevatorState")
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

    Logger.processInputs("Elevator", inputs);

    if (disableSupplier.getAsBoolean() || elevatorState == ElevatorState.STOP) {
      io.stop();
    }

    setBrakeMode(!coastSupplier.getAsBoolean() || DriverStation.isEnabled());
    // LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    // LoggedTunableNumber.ifChanged(
    //     hashCode(), kSVA -> io.setFF(kSVA[0], kSVA[1], kSVA[2], kSVA[3]), kS, kV, kA, kG);

    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     kSVA -> io.setMotionMagicCruise(kSVA[0], kSVA[1], kSVA[2]),
    //     cruiseV,
    //     cruiseA,
    //     cruiseJ);

    //visualizer.updateVisualizer(inputs.positionRads);
    if (!characterizing
        && brakeModeEnabled
        && !disableSupplier.getAsBoolean()
        && elevatorState != ElevatorState.STOP) {

      io.setPosition(elevatorState.getRads());
    }
    Logger.recordOutput(
        "Superstructure/ShooterArm/ArmSetpoint", Units.radiansToDegrees(elevatorState.getRads()));
  }

  public void setArmPosition(DoubleSupplier desiredArmPosition) {
    io.setPosition(Units.degreesToRadians(desiredArmPosition.getAsDouble()));
  }

  // @AutoLogOutput(key = "Superstructure/ShooterArm/AtGoal")
  // public boolean atGoal() {
  //   return EqualsUtil.epsilonEquals(inputs.positionRad, elevatorState.getRads(), armTolerance);
  // }


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



}
