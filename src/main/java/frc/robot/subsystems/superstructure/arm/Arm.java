package frc.robot.subsystems.superstructure.arm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Arm {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIoInputsAutoLogged();
    private final ArmVisualizer visualizer = new ArmVisualizer();
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
    @RequiredArgsConstructor
    public enum ArmState{
        STOP(),
        STOW(),
        ALGL(),
        ALGH(),
        CLVL1(),
        CLVL2(),
        ClVL3(),
        CLVL4(),
        INTAKE(),
        PROCESS(),
        BARGE();
        private  DoubleSupplier armSetpointSupplier;

        private double getRads(){
            return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
        }
    }
    @AutoLogOutput(key = "Superstructure/Arm/ArmState")
    @Getter
    @Setter
    private ArmState armState = ArmState.STOW;

    public Arm(ArmIO io) {
      this.io = io;
    }
}