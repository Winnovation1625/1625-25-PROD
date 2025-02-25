package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.Arm.ArmState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.Wrist.WristState;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  private final Arm arm;
  private final Elevator elevator;
  private final Wrist wrist;
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("setpoint");
  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("measured");

  @AutoLogOutput @Getter
  private SuperstructureStates superstructureGoal = SuperstructureStates.STOW;

  public enum SuperstructureStates {
    INTAKING,
    STOW,
    CLIMB,
    BARGE,
    CLVL1,
    CLVL2,
    CLVL3,
    CLVL4,
    ALVL2,
    ALVL3,
    PROCESS;
  }

  public Superstructure(Arm arm, Elevator elevator, Wrist wrist) {
    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;
    setDefaultCommand(setGoalCommand(SuperstructureStates.STOW));
  }

  @Override
  public void periodic() {
    arm.periodic();
    elevator.periodic();
    wrist.periodic();
    switch (superstructureGoal) {
      case INTAKING -> {
        arm.setArmState(ArmState.INTAKE);
        elevator.setElevatorState(ElevatorState.INTAKE);
        wrist.setWristState(WristState.IDLE);
      }

      case STOW -> {
        arm.setArmState(ArmState.STOW);
        elevator.setElevatorState(ElevatorState.STOW);
        wrist.setWristState(WristState.IDLE);
      }

      case CLIMB -> {
        arm.setArmState(ArmState.STOW);
        elevator.setElevatorState(ElevatorState.STOW);
        wrist.setWristState(WristState.IDLE);
      }

      case BARGE -> {
        arm.setArmState(ArmState.BARGE);
        elevator.setElevatorState(ElevatorState.BARGE);
        wrist.setWristState(WristState.IDLE);
      }

      case CLVL1 -> {
        arm.setArmState(ArmState.CLVL1);
        elevator.setElevatorState(ElevatorState.CLVL1);
        wrist.setWristState(WristState.TROUGH);
      }

      case CLVL2 -> {
        arm.setArmState(ArmState.CLVL2);
        elevator.setElevatorState(ElevatorState.CLVL2);
        wrist.setWristState(WristState.LEVEL_TWO);
      }

      case CLVL3 -> {
        arm.setArmState(ArmState.CLVL3);
        elevator.setElevatorState(ElevatorState.CLVL3);
        wrist.setWristState(WristState.LEVEL_THREE);
      }

      case CLVL4 -> {
        arm.setArmState(ArmState.CLVL4);
        elevator.setElevatorState(ElevatorState.CLVL4);
        wrist.setWristState(WristState.LEVEL_FOUR);
      }

      case ALVL2 -> {
        arm.setArmState(ArmState.ALVL2);
        elevator.setElevatorState(ElevatorState.ALVL2);
        wrist.setWristState(WristState.IDLE);
      }

      case ALVL3 -> {
        arm.setArmState(ArmState.ALVL3);
        elevator.setElevatorState(ElevatorState.ALVL3);
        wrist.setWristState(WristState.IDLE);
      }

      case PROCESS -> {
        arm.setArmState(ArmState.PROCESS);
        elevator.setElevatorState(ElevatorState.PROCESS);
        wrist.setWristState(WristState.IDLE);
      }
    }
    setpointVisualizer.updateSuperstructurePose(
        elevator.getElevatorState().getElevatorSetpointFromGround().getAsDouble(),
        arm.getArmState().getArmSetpointSupplier().getAsDouble(),
        wrist.getWristState().getWristSetpointSupplier().getAsDouble());
    measuredVisualizer.updateSuperstructurePose(
        elevator.getElevatorHeight(), arm.getArmPos(), wrist.getWristPos());
  }

  public void setGoal(SuperstructureStates desiredState) {
    if (desiredState != superstructureGoal) {
      superstructureGoal = desiredState;
    }
  }

  public Command setGoalCommand(SuperstructureStates goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(SuperstructureStates.STOW))
        .withName("Superstructure " + goal);
  }

  public boolean atArmGoal() {
    return wrist.atGoal() && arm.atGoal() && elevator.atGoal();
  }
}
