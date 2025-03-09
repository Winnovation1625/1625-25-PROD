package frc.robot.subsystem.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.superstructure.arm.Arm;
import frc.robot.subsystem.superstructure.arm.Arm.ArmState;
import frc.robot.subsystem.superstructure.elevator.Elevator;
import frc.robot.subsystem.superstructure.elevator.Elevator.ElevatorState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  private final Arm arm;
  private final Elevator elevator;
  private final Supplier<Boolean> hasAlgaeSupplier;
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("setpoint");
  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("measured");

  private LoggedTunableNumber elevatorThreshold =
      new LoggedTunableNumber("Superstructure/ElevatorThreshold", Units.inchesToMeters(27.5591));
  private LoggedTunableNumber elevatorAlgaeThreshold =
      new LoggedTunableNumber("Superstructure/ElevatorAlgaeThreshold", Units.inchesToMeters(40.0));

  @AutoLogOutput @Getter
  private SuperstructureStates superstructureGoal = SuperstructureStates.STOP;

  @AutoLogOutput private ElevatorState elevatorState = ElevatorState.STOP;
  @AutoLogOutput private ArmState armState = ArmState.STOP;

  // TODO: Make these hold the elevator and arm positions in this state, eliminate it in the arm and
  // elevator classes
  @RequiredArgsConstructor
  public enum SuperstructureStates {
    INTAKING(ElevatorState.INTAKING, ArmState.INTAKING),
    STOW(ElevatorState.STOW, ArmState.STOW),
    ALGAE_STOW(ElevatorState.ALGAE_STOW, ArmState.STOW),
    ALGAE_CLEARANCE(ElevatorState.ALGAE_CLEARANCE, ArmState.STOW),
    CLIMB(ElevatorState.CLIMB, ArmState.CLIMB),
    BARGE(ElevatorState.BARGE, ArmState.BARGE),
    CLVL2(ElevatorState.CLVL2, ArmState.CLVL2),
    TROUGH(ElevatorState.TROUGH, ArmState.TROUGH),
    CLVL3(ElevatorState.CLVL3, ArmState.CLVL3),
    CLVL4(ElevatorState.CLVL4, ArmState.CLVL4),
    ALVL2(ElevatorState.ALVL2, ArmState.ALVL2),
    ALVL3(ElevatorState.ALVL3, ArmState.ALVL3),
    PROCESS(ElevatorState.PROCESS, ArmState.PROCESS),
    STOP(ElevatorState.STOP, ArmState.STOP);
    @Getter private final ElevatorState elevatorState;
    @Getter private final ArmState armState;
  }

  public Superstructure(Arm arm, Elevator elevator, Supplier<Boolean> hasAlgaeSupplier) {
    this.arm = arm;
    this.elevator = elevator;
    this.hasAlgaeSupplier = hasAlgaeSupplier;
    setSuperstructureCommand(SuperstructureStates.STOW).schedule();
  }

  @Getter private SuperstructureStates previousState = SuperstructureStates.STOW;

  @Override
  public void periodic() {
    arm.periodic();
    elevator.periodic();
    // switch (superstructureGoal) {
    //   case INTAKING -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case STOW -> {
    //     buildSuperStructureCommand(superstructureGoal, previousState);
    //   }

    //   case CLIMB -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case BARGE -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case TROUGH -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case CLVL2 -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //     setHasAlgae(true);
    //   }

    //   case CLVL3 -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case CLVL4 -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case ALVL2 -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case ALVL3 -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }

    //   case PROCESS -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }
    //   case STOP -> {
    //     buildSuperStructureCommand(previousState, superstructureGoal);
    //   }
    // }
    setpointVisualizer.updateSuperstructurePose(
        elevatorState.getElevatorHeight().getAsDouble(), armState.getArmAngle().getAsDouble());
    measuredVisualizer.updateSuperstructurePose(elevator.getElevatorHeight(), arm.getArmPos());
  }

  // public void setGoal(SuperstructureStates desiredState) {
  //   if (desiredState != superstructureGoal) {
  //     superstructureGoal = desiredState;
  //   }
  // }

  public Command setSuperstructureCommand(SuperstructureStates to) {

    SuperstructureStates intermediateState =
        hasAlgaeSupplier.get() ? SuperstructureStates.ALGAE_CLEARANCE : SuperstructureStates.STOW;
    double threshold =
        hasAlgaeSupplier.get() ? elevatorThreshold.get() : elevatorAlgaeThreshold.get();
    if (superstructureGoal == to) {
      return Commands.none();
    }

    if (elevatorState.getElevatorHeight().getAsDouble() < threshold
        && to.elevatorState.getElevatorHeight().getAsDouble() < threshold) {
      return runElevatorToState(intermediateState.getElevatorState())
          .andThen(Commands.waitUntil(() -> atGoal()))
          .andThen(runArmToState(to.getArmState()))
          .andThen(Commands.waitUntil(() -> atGoal()))
          .andThen(runElevatorToState(to.getElevatorState()))
          .andThen(() -> superstructureGoal = to, this);
      // add intermediate state
    } else if (elevatorState.getElevatorHeight().getAsDouble() < threshold
        && to.elevatorState.getElevatorHeight().getAsDouble() > threshold) {
      return runElevatorToState(intermediateState.getElevatorState())
          .andThen(Commands.waitUntil(() -> atGoal()))
          .andThen(runArmToState(to.getArmState()))
          .alongWith(runElevatorToState(to.getElevatorState()))
          .andThen(() -> superstructureGoal = to, this);
      // add intermediate state
    } else if (elevatorState.getElevatorHeight().getAsDouble() > threshold
        && to.elevatorState.getElevatorHeight().getAsDouble() < threshold) {
      return runElevatorToState(intermediateState.getElevatorState())
          .alongWith(runArmToState(to.getArmState()))
          .andThen(Commands.waitUntil(() -> atGoal()))
          .andThen(runElevatorToState(to.getElevatorState()))
          .andThen(() -> superstructureGoal = to, this);
      // add intermediate state
    } else {
      return runElevatorToState(to.elevatorState)
          .alongWith(runArmToState(to.armState))
          .andThen(() -> superstructureGoal = to, this); // ends immediately,
      // no intermediate state continue as nomal
    }
  }

  private Command runArmToState(ArmState armState) {
    this.armState = armState;
    return Commands.runOnce(() -> arm.setPosition(armState.getArmAngle()), this);
  }

  // public Command setGoalCommand(SuperstructureStates goal) {
  //   return startEnd(() -> setGoal(goal), () -> setGoal(SuperstructureStates.STOW))
  //       .withName("Superstructure " + goal);
  // }

  private Command runElevatorToState(ElevatorState state) {
    this.elevatorState = state;
    return Commands.runOnce(() -> elevator.setPosition(state.getElevatorHeight()), this);
  }

  public boolean atGoal() {
    return arm.atGoal() && elevator.atGoal();
  }

  public boolean atSuperStructureGoal() {
    return atGoal()
        && superstructureGoal.getArmState() == armState
        && superstructureGoal.getElevatorState() == elevatorState;
  }

  public boolean elevatorClearofBumpers() {
    return elevator.getElevatorHeight() > elevatorThreshold.get();
  }
}
