package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.Arm.ArmState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorState;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  private final Arm arm;
  private final Elevator elevator;
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("setpoint");
  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("measured");

  @AutoLogOutput @Getter
  private SuperstructureStates superstructureGoal = SuperstructureStates.STOW;

  // TODO: Make these hold the elevator and arm positions in this state, eliminate it in the arm and
  // elevator classes
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

  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
    setDefaultCommand(setGoalCommand(SuperstructureStates.STOW));
  }

  @Override
  public void periodic() {
    arm.periodic();
    elevator.periodic();
    switch (superstructureGoal) {
      case INTAKING -> {
        arm.setArmState(ArmState.INTAKE);
        elevator.setElevatorState(ElevatorState.INTAKE);
      }

      case STOW -> {
        arm.setArmState(ArmState.STOW);
        elevator.setElevatorState(ElevatorState.STOW);
      }

      case CLIMB -> {
        arm.setArmState(ArmState.STOW);
        elevator.setElevatorState(ElevatorState.STOW);
      }

      case BARGE -> {
        arm.setArmState(ArmState.BARGE);
        elevator.setElevatorState(ElevatorState.BARGE);
      }

      case CLVL1 -> {
        arm.setArmState(ArmState.CLVL1);
        elevator.setElevatorState(ElevatorState.CLVL1);
      }

      case CLVL2 -> {
        arm.setArmState(ArmState.CLVL2);
        elevator.setElevatorState(ElevatorState.CLVL2);
      }

      case CLVL3 -> {
        arm.setArmState(ArmState.CLVL3);
        elevator.setElevatorState(ElevatorState.CLVL3);
      }

      case CLVL4 -> {
        arm.setArmState(ArmState.CLVL4);
        elevator.setElevatorState(ElevatorState.CLVL4);
      }

      case ALVL2 -> {
        arm.setArmState(ArmState.ALVL2);
        elevator.setElevatorState(ElevatorState.ALVL2);
      }

      case ALVL3 -> {
        arm.setArmState(ArmState.ALVL3);
        elevator.setElevatorState(ElevatorState.ALVL3);
      }

      case PROCESS -> {
        arm.setArmState(ArmState.PROCESS);
        elevator.setElevatorState(ElevatorState.PROCESS);
      }
    }
    setpointVisualizer.updateSuperstructurePose(
        elevator.getElevatorState().getElevatorSetpointFromGround().getAsDouble(),
        arm.getArmState().getArmSetpointSupplier().getAsDouble());
    measuredVisualizer.updateSuperstructurePose(elevator.getElevatorHeight(), arm.getArmPos());
  }

  public void setGoal(SuperstructureStates desiredState) {
    if (desiredState != superstructureGoal) {
      superstructureGoal = desiredState;
    }
  }

  public Command buildSuperStructureCommand(SuperstructureStates from, SuperstructureStates to) {
    return null;
    /* https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html#motion-profiling
    * https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/subsystems/superstructure/Superstructure.java#L536
    *
    * this command will have a series of checks to determine if the path is going to collide with anything
    *
    * Stow position should be set so that the arm can fully rotate without hitting anything without an algae, we will need a different stow for when we have an algae
    *
    * drivebase check will be to check if the arm is going to collide with the drivebase
    *    if it is then we will add an intermediate step that will require us to get to a height that it will not collide with the drivebase and then we can go to our desired positions
    * if the elevator isn't going trying to go below a certain height then we wont need to worry about the arm colliding with the drivebase and can skip the check
    *
    * cases to worry about when elevator is above the threshold height and wanting to go below:
    *   - if currently above the threshold height allow the arm to rotate to its desired position while setting the elevator to stow position, once the arm is within tolerance then we can move the elevator to the desired position
    * cases to worry about when elevator is below the threshold height and wanting to go above:
    *   - if currently below the threshold height that is safe to rotate the arm, move the elevator to stow position before allowing arm rotation, one past stow position allow the arm to rotate to its desired position
    * cases to worry about when elevator is below the threshold height and wanting to go to a different arm position below the threshold height:
    *   - if the desired state is to keep the elevator below the threshold height, we need to move to stow position before allowing the arm to rotate, once in stow allow the arm to reach it's desired position before moving down to the desired elevator position
    *
    * chatGPT cleaned this up in a more readable manor:
     function moveArmAndElevator(desiredArmPos, desiredElevatorPos, hasAlgae) {
     // Set the appropriate stow position based on whether we have algae
     stowPos = hasAlgae ? algaeStowPos : normalStowPos

     // Case 1: Elevator is above threshold and moving downward
     if (elevator.currentHeight > THRESHOLD and desiredElevatorPos < THRESHOLD) {
         // Allow arm to rotate while moving elevator to stow position first.
         elevator.moveTo(stowPos)
         moveArmTo(desiredArmPos)
         if (armWithinTolerance()) {
             elevator.moveTo(desiredElevatorPos)
         }
     }
     // Case 2: Elevator is below threshold and moving upward
     else if (elevator.currentHeight < THRESHOLD and desiredElevatorPos > THRESHOLD) {
         // Move elevator to stow position to avoid drivebase collision,
         // then rotate the arm and finally move elevator to desired position.
         elevator.moveTo(stowPos)
         if(elevatorClearofBumpers()) {
           moveArmTo(desiredArmPos)
           elevator.moveTo(desiredElevatorPos)
       }
     }
     // Case 3: Elevator remains below threshold
     else if (elevator.currentHeight < THRESHOLD and desiredElevatorPos < THRESHOLD) {
         // Even if both positions are below the threshold, use stow to clear any obstacles.
         elevator.moveTo(stowPos)
         if(elevatorClearofBumpers()) {
           moveArmTo(desiredArmPos)
           if (armWithinTolerance()) {
             elevator.moveTo(desiredElevatorPos)
           }
         }

     }
     // Default case: Other movements
         else {
             // No collision risk, perform direct movements
             moveArmTo(desiredArmPos)
             elevator.moveTo(desiredElevatorPos)
         }
       }
     }
    * second check will be for when we have an algae and need to worry about it colliding with the elevator and knocking the algae out, or worse...
    */
    //
  }

  public Command setGoalCommand(SuperstructureStates goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(SuperstructureStates.STOW))
        .withName("Superstructure " + goal);
  }

  public boolean atArmGoal() {
    return arm.atGoal() && elevator.atGoal();
  }
}
