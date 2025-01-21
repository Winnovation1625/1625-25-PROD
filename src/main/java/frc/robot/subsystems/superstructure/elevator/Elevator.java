package frc.robot.subsystems.superstructure.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Elevator {

   private final ElevatorIO io;
   private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
   private final ElevatorVisualizer visualizer =
    new ElevatorVisualizer(ElevatorConstants.elevatorPose);

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

  @RequiredArgsConstructor
  public enum ElevatorState {

    STOP(),   //Will discuss about values.
    STOW(),
    LEVEL3(),
    LEVEL4(),
    BARGE();

    private DoubleSupplier elevatorSetpointSupplier;

    private double getRads(){
        return Units.degreesToRadians(elevatorSetpointSupplier.getAsDouble());
    }

  }

  @AutoLogOutput(key = "Superstructure/ElevatorArm/ElevatorState")
  @Getter
  @Setter
  private ElevatorState elevatorState = ElevatorState.STOW;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }





}
