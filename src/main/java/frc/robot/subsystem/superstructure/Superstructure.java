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
    setpointVisualizer.updateSuperstructurePose(
        elevatorState.getElevatorHeight().getAsDouble(), armState.getArmAngle().getAsDouble());
    measuredVisualizer.updateSuperstructurePose(elevator.getElevatorHeight(), arm.getArmPos());
  }

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
            superstructureGoal == SuperstructureStates.CLVL3
                || superstructureGoal == SuperstructureStates.CLVL2
                || superstructureGoal == SuperstructureStates.ALVL2
                || superstructureGoal == SuperstructureStates.ALVL3;
    BooleanSupplier comingFromCoral4Score = () -> superstructureGoal == SuperstructureStates.CLVL4;
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
                        Commands.waitTime(Seconds.of(0.5))
                            .andThen(runElevatorToState(toElevatorState)))
                    .andThen(() -> superstructureGoal = to.get(), this)
                    .andThen(Commands.print("Best Case Superstructure Run")),
                Commands.either(
                    runArmToState(() -> ArmState.CLVL3)
                        .andThen(Commands.waitUntil(() -> arm.atGoal()))
                        .andThen(
                            runArmToState(() -> to.get().armState)
                                .alongWith(
                                    Commands.waitTime(Seconds.of(0.5))
                                        .andThen(runElevatorToState(toElevatorState))))
                        .andThen(() -> superstructureGoal = to.get(), this)
                        .andThen(Commands.print("Best Case Superstructure Run")),
                    runElevatorToState(toElevatorState)
                        .alongWith(runArmToState(() -> to.get().armState))
                        .andThen(() -> superstructureGoal = to.get())
                        .andThen(Commands.print("Best Case Superstructure Run")),
                    comingFromCoral4Score),
                comingFromCoralScore),
            goingBelowThreshold),
        currentlyBelowThreshold);
  }

  public Command leaveStartConfigToGoal(SuperstructureStates goal) {
    return Commands.either(
        runElevatorToState(() -> ElevatorState.ALGAE_CLEARANCE)
            .andThen(Commands.waitUntil(() -> atGoal()))
            .andThen(Commands.print("Elevator at algae Clearance, moving Arm"))
            .andThen(runArmToState(() -> goal.getArmState()))
            .andThen(Commands.waitUntil(() -> atGoal()))
            .andThen(Commands.print("Arm at target, moving elevator"))
            .andThen(runElevatorToState(() -> goal.getElevatorState()))
            .andThen(() -> superstructureGoal = goal)
            .andThen(Commands.print("Worst Case Superstructure Run From Start Config")),
        runElevatorToState(() -> ElevatorState.ALGAE_CLEARANCE)
            .andThen(Commands.waitUntil(() -> atGoal()))
            .andThen(
                runArmToState(() -> goal.getArmState())
                    .alongWith(runElevatorToState(() -> goal.getElevatorState())))
            .andThen(() -> superstructureGoal = goal)
            .andThen(Commands.print("From Under Superstructure Run From Start Config")),
        () -> goal == SuperstructureStates.TROUGH);
  }

  public Command leaveStartConfigToStow() {
    return runElevatorToState(() -> ElevatorState.ALGAE_CLEARANCE)
        .andThen(Commands.waitUntil(() -> atGoal()))
        .andThen(Commands.print("Elevator at algae Clearance, moving Arm"))
        .andThen(runArmToState(() -> SuperstructureStates.STOW.getArmState()))
        .andThen(Commands.waitUntil(() -> atGoal()))
        .andThen(Commands.print("Arm at target, moving elevator"))
        .andThen(runElevatorToState(() -> SuperstructureStates.STOW.getElevatorState()))
        .andThen(() -> superstructureGoal = SuperstructureStates.STOW)
        .andThen(Commands.print("Going to Stow From Start Config"));
  }

  public Command runArmToState(Supplier<ArmState> armState) {
    return Commands.runOnce(() -> arm.setPosition(armState.get()))
        .andThen(Commands.runOnce(() -> this.armState = armState.get()));
  }

  public Command runElevatorToState(Supplier<ElevatorState> state) {
    return Commands.runOnce(() -> elevator.setPosition(state.get()))
        .andThen(Commands.runOnce(() -> elevatorState = state.get()));
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

  public void setElevatorEncoderPosition(double encoderSetpoint) {
    elevator.setEncoderPosition(encoderSetpoint);
  }
}
