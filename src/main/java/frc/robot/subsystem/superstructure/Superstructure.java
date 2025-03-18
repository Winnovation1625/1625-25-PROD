package frc.robot.subsystem.superstructure;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.superstructure.arm.Arm;
import frc.robot.subsystem.superstructure.arm.Arm.ArmState;
import frc.robot.subsystem.superstructure.elevator.Elevator;
import frc.robot.subsystem.superstructure.elevator.Elevator.ElevatorState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
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
      new LoggedTunableNumber("Superstructure/ElevatorThreshold", 0.259);
  private LoggedTunableNumber elevatorAlgaeThreshold =
      new LoggedTunableNumber("Superstructure/ElevatorAlgaeThreshold", 0.479);

  @AutoLogOutput @Getter
  private SuperstructureStates superstructureGoal = SuperstructureStates.STOP;

  @AutoLogOutput private ElevatorState elevatorState = ElevatorState.STOP;
  @AutoLogOutput private ArmState armState = ArmState.STOP;

  // TODO: Make these hold the elevator and arm positions in this state, eliminate it in the arm and
  // elevator classes
  @RequiredArgsConstructor
  public enum SuperstructureStates {
    ALGAE_INTAKING(ElevatorState.ALGAE_INTAKING, ArmState.ALGAE_INTAKING),
    CORAL_INTAKING(ElevatorState.CORAL_INTAKING, ArmState.CORAL_INTAKING),
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
  }

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

  public Command setSuperstructureCommand(Supplier<SuperstructureStates> to) {
    Supplier<ElevatorState> toElevatorState =
        () ->
            to.get() == SuperstructureStates.STOW && hasAlgaeSupplier.get()
                ? ElevatorState.ALGAE_STOW
                : to.get().getElevatorState();
    BooleanSupplier currentlyBelowThreshold =
        () ->
            elevatorState.getElevatorHeight().getAsDouble()
                < (hasAlgaeSupplier.get() ? elevatorAlgaeThreshold.get() : elevatorThreshold.get());
    BooleanSupplier goingBelowThreshold =
        () ->
            toElevatorState.get().getElevatorHeight().getAsDouble()
                < (hasAlgaeSupplier.get() ? elevatorAlgaeThreshold.get() : elevatorThreshold.get());
    Supplier<ElevatorState> intermediateState =
        () -> hasAlgaeSupplier.get() ? ElevatorState.ALGAE_CLEARANCE : ElevatorState.STOW;
    BooleanSupplier comingFromCoralScore =
        () ->
            superstructureGoal == SuperstructureStates.CLVL4
                || superstructureGoal == SuperstructureStates.CLVL3
                || superstructureGoal == SuperstructureStates.CLVL2
                || superstructureGoal == SuperstructureStates.ALVL2
                || superstructureGoal == SuperstructureStates.ALVL3;
    return Commands.either(
        Commands.either( // true, true worst case
            runElevatorToState(intermediateState)
                .andThen(Commands.waitUntil(() -> atGoal()))
                .andThen(runArmToState(() -> to.get().getArmState()))
                .andThen(Commands.waitUntil(() -> atGoal()))
                .andThen(runElevatorToState(toElevatorState))
                .andThen(() -> superstructureGoal = to.get())
                .andThen(Commands.print("Worst Case Superstructure Run")),
            // true, false starting below thresh, but going above
            runElevatorToState(intermediateState)
                .andThen(Commands.waitUntil(() -> atGoal()))
                .andThen(
                    runArmToState(() -> to.get().getArmState())
                        .alongWith(runElevatorToState(toElevatorState)))
                .andThen(() -> superstructureGoal = to.get())
                .andThen(Commands.print("From Under Superstructure Run")),
            goingBelowThreshold),
        Commands.either(
            // false, true above thresh going below
            runElevatorToState(intermediateState)
                .alongWith(runArmToState(() -> to.get().getArmState()))
                .andThen(Commands.waitUntil(() -> atGoal()))
                .andThen(runElevatorToState(toElevatorState))
                .andThen(() -> superstructureGoal = to.get())
                .andThen(Commands.print("From Above Case Superstructure Run")),
            // false false no issues with threshold
            Commands.either(
                runArmToState(() -> to.get().armState)
                    .alongWith(
                        Commands.waitTime(Seconds.of(1))
                            .andThen(runElevatorToState(toElevatorState)))
                    .andThen(() -> superstructureGoal = to.get(), this)
                    .andThen(Commands.print("Best Case Superstructure Run")),
                runElevatorToState(toElevatorState)
                    .alongWith(runArmToState(() -> to.get().armState))
                    .andThen(() -> superstructureGoal = to.get())
                    .andThen(Commands.print("Best Case Superstructure Run")),
                comingFromCoralScore),
            goingBelowThreshold),
        currentlyBelowThreshold);

    // if (elevatorState.getElevatorHeight().getAsDouble() < threshold
    //     && to.elevatorState.getElevatorHeight().getAsDouble() < threshold) {
    //   System.out.println("Worst Case Scenario");
    //   return runElevatorToState(intermediateState.getElevatorState())
    //       .andThen(Commands.waitUntil(() -> atGoal()))
    //       .andThen(runArmToState(to.getArmState()))
    //       .andThen(Commands.waitUntil(() -> atGoal()))
    //       .andThen(runElevatorToState(to.getElevatorState()))
    //       .andThen(() -> superstructureGoal = to, this)
    //       .withName("Worst Case Superstructure Run");
    //   // add intermediate state
    // } else if (elevatorState.getElevatorHeight().getAsDouble() < threshold
    //     && to.elevatorState.getElevatorHeight().getAsDouble() > threshold) {
    //   System.out.println("from below Case Scenario");
    //   return runElevatorToState(intermediateState.getElevatorState())
    //       .andThen(Commands.waitUntil(() -> atGoal()))
    //       .andThen(runArmToState(to.getArmState()))
    //       .alongWith(runElevatorToState(to.getElevatorState()))
    //       .andThen(() -> superstructureGoal = to, this)
    //       .withName("From Under Superstructure Run");
    //   // add intermediate state
    // } else if (elevatorState.getElevatorHeight().getAsDouble() > threshold
    //     && to.elevatorState.getElevatorHeight().getAsDouble() < threshold) {
    //   System.out.println("from above Case Scenario");
    //   return runElevatorToState(intermediateState.getElevatorState())
    //       .alongWith(runArmToState(to.getArmState()))
    //       .andThen(Commands.waitUntil(() -> atGoal()))
    //       .andThen(runElevatorToState(to.getElevatorState()))
    //       .andThen(() -> superstructureGoal = to, this)
    //       .withName("From Above Case Superstructure Run");
    //   // add intermediate state
    // } else {
    //   System.out.println("Best Case Scenario");
    //   return runElevatorToState(to.elevatorState)
    //       .alongWith(runArmToState(to.armState))
    //       .andThen(() -> superstructureGoal = to, this)
    //       .withName("Best Case Superstructure Run"); // ends immediately,
    //   // no intermediate state continue as nomal
    // }
  }

  public Command runArmToState(Supplier<ArmState> armState) {
    return Commands.runOnce(() -> arm.setPosition(armState.get()))
        .alongWith(Commands.runOnce(() -> this.armState = armState.get()));
  }

  // public Command setGoalCommand(SuperstructureStates goal) {
  //   return startEnd(() -> setGoal(goal), () -> setGoal(SuperstructureStates.STOW))
  //       .withName("Superstructure " + goal);
  // }

  public Command runElevatorToState(Supplier<ElevatorState> state) {
    return Commands.runOnce(() -> elevator.setPosition(state.get()))
        .alongWith(Commands.runOnce(() -> elevatorState = state.get()));
  }

  public boolean atGoal() {
    return arm.atGoal() && elevator.atGoal();
  }

  public boolean atSuperStructureGoal() {
    return atGoal()
        && superstructureGoal.getArmState() == armState
        && (superstructureGoal.getElevatorState() == elevatorState
            || (superstructureGoal.getElevatorState() == ElevatorState.STOW
                && elevatorState == ElevatorState.ALGAE_STOW));
  }

  public boolean elevatorClearofBumpers() {
    return elevator.getElevatorHeight() > elevatorThreshold.get();
  }

  public boolean atElevatorGoal() {
    return elevator.atGoal();
  }

  public boolean atArmGoal() {
    return arm.atGoal();
  }
}
